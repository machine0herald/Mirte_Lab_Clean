import numpy as np
import cv2

import rclpy

# from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator
# import mirte_lc_nav2.utils as ut
# from mirte_lc_nav2.utils import LogType
from navigator_types import SystematicNavigator, ReactiveNavigator
import utils as ut
from utils import LogType

import trajgenpy as tjp
from trajgenpy import Geometries

import shapely
from shapely.validation import explain_validity
from shapely.geometry.polygon import orient

from scipy.spatial import KDTree
import networkx as nx
from skimage.morphology import skeletonize, medial_axis, thin, max_tree, binary_closing
from scipy.interpolate import interp1d


class StraightLinePath(SystematicNavigator):
    name = "StraightLinePlanner"

    def __init__(self, node=None, resolution=0.1, length=2.0):
        super().__init__(node, resolution)
        self.length = length
        self.start_pose = (0, 0, 0)

    def generate_path(self):
        if self.start_pose is None:
            return None

        x0, y0, yaw = self.start_pose
        n_points = int(self.length / self.resolution)

        path = []
        for i in range(n_points):
            d = i * self.resolution
            path.append([x0 + d, y0 + d, yaw])

        self.paths = np.array([path])


class BousPath(SystematicNavigator):
    name = "BousPlanner"

    def __init__(self, node=None, resolution=0.1):
        self.node = node
        super().__init__(node, resolution)

    def bcd(self, polygons):
        """
        Takes a list of polygons, polygon at index 0 is the outer polygon, and the rest are holes.
        Decomposes the polygon into a list of convex polygons.
        """

        if self.node is not None:
            ut.log(self.node, LogType.INFO, f"Contours: {len(polygons)}")

        for i, c in enumerate(polygons):
            ut.log(self.node, LogType.INFO, f"{i}: {len(c)} points")

        # polygons = self.polymap.copy()
        outer_poly = shapely.Polygon(polygons[0])

        if not outer_poly.is_valid:
            ut.log(
                self.node,
                LogType.WARN,
                f"Invalid outer polygon: {explain_validity(outer_poly)}",
            )
            outer_poly = outer_poly.buffer(0)

            if not outer_poly.is_valid:
                ut.log(self.node, LogType.WARN, "Could not fix outer polygon")
                return False

            if outer_poly.is_empty:
                ut.log(self.node, LogType.ERR, "Outer polygon is empty after fix")
                return False

            if outer_poly.geom_type == "MultiPolygon":
                ut.log(
                    self.node,
                    LogType.WARN,
                    "Outer polygon became MultiPolygon, taking largest piece",
                )
                outer_poly = max(outer_poly.geoms, key=lambda p: p.area)

        outer = outer_poly
        polygons.pop(0)

        holes = []

        for contour in polygons:
            poly = shapely.Polygon(contour).simplify(0.1, preserve_topology=True)

            if not poly.is_valid:
                ut.log(
                    self.node,
                    LogType.WARN,
                    f"Invalid polygon: {explain_validity(poly)}",
                )
                poly = poly.buffer(0)

                if not poly.is_valid:
                    ut.log(self.node, LogType.WARN, "Could not fix polygon")
                    continue

                if poly.is_empty:
                    ut.log(self.node, LogType.ERR, "Polygon is empty after fix")
                    return False

                if poly.geom_type == "MultiPolygon":
                    ut.log(
                        self.node,
                        LogType.WARN,
                        "Polygon became MultiPolygon, taking largest piece",
                    )
                    poly = max(poly.geoms, key=lambda p: p.area)

            holes.append(poly)
        obstacles = shapely.MultiPolygon(holes)

        ut.log(self.node, LogType.INFO, "Performing Decomposition")
        polygon_list = Geometries.decompose_polygon(outer, obstacles=obstacles)
        polygon_list
        self.raw_cells = [
            np.array(polygon.exterior.coords, dtype=np.int32)
            for polygon in polygon_list
        ]

        ut.log(self.node, LogType.INFO, "map decomposed, publishing decomposition")
        if self.node is not None:
            self.publish_decomposition()

        return polygon_list

    def bous_path(self, cells, robot_width=0.3):
        ut.log(self.node, LogType.INFO, f"Number of cells: {len(cells)}")

        offset = Geometries.get_sweep_offset(overlap=0.0, height=0.6, field_of_view=90)

        ut.log(self.node, LogType.INFO, "generating full trajectory")
        for cell in cells:
            cell = orient(cell, sign=1.0)
            sweeps = Geometries.generate_sweep_pattern(
                cell, offset, clockwise=False, connect_sweeps=True
            )
            paths_mls = Geometries.GeoMultiTrajectory(sweeps).get_geometry()
            self.paths.append(self.multiline_to_coords(paths_mls))
            # result.extend(sweeps)

        # mls = Geometries.GeoMultiTrajectory(result).get_geometry()
        # self.resultant_path = self.multiline_to_coords(mls)
        if self.node is not None:
            self.publish_path()

        # path_graph = []

        # for path in self.paths:
        #     graph = self.set_waypoints(waypoints=path, resolution=self.resolution)
        #     leaf_nodes = self.find_leaf_nodes(graph)
        #     path_graph.append((graph, leaf_nodes))

        # visited_paths = set()
        # overall_path = []

        # current_position = np.array(self.start)

    def multiline_to_coords(self, multiline):
        coords = []
        if isinstance(multiline, shapely.MultiLineString):
            for line in multiline.geoms:
                coords.extend(list(line.coords))
        else:
            for line in multiline:
                coords.extend(list(line.coords))

        return coords

    def generate_path(self):
        self.paths = []
        self.cells = []
        for group in self.polymap:
            cells = self.bcd(group)
            self.cells.extend(cells)
            self.bous_path(cells)


class SkeletonPath(SystematicNavigator):
    name = "SkeletonPlanner"

    def __init__(self, node=None, resolution=0.1):
        super().__init__(node, resolution)
        self.path = None
        self.leaf_nodes = []
        self.offset = 1

    def read(self, plot=False) -> np.ndarray:
        """
        Generates skeleton waypoints from polygon groups.
        """
        #TODO: WAYPOINT GROUP 1 HAS A COPY OF WAYPOINT GROUP 0 IN IT

        self.waypoint_groups = []

        # Create a contour map to store the filled contours for skeletonization
        self.contour_map = np.zeros_like(self.map, dtype=np.uint8) # Used for visualization
        self.origin = np.array([self.map_height / 2, self.map_width / 2])

        for group in self.polymap:
            contour_map = np.zeros_like(self.map, dtype=np.uint8)

            # Draw filled contours
            for cont in group:
                contour = np.asarray(cont, dtype=np.float32)

                # Convert world coordinates -> pixels
                contour = self.world_to_pixel(contour)
                contour = contour.reshape((-1, 1, 2)).astype(np.int32)
                
                self.contour_map = cv2.drawContours(self.contour_map, [contour], -1, 255, -1)
                local_contour_map = cv2.drawContours(contour_map, [contour], -1, 255, -1)

            # Skeletonize expects bool image
            skeleton_map = medial_axis(local_contour_map > 0,
                                        return_distance=False)
            # skeleton_map = skeletonize(local_contour_map > 0)
            self.skeleton_map = skeleton_map.astype(np.uint8) * 255

            # Get skeleton pixels
            skeleton_points = np.column_stack(np.where(skeleton_map))

            # Convert pixel coordinates -> world coordinates
            waypoints = np.zeros((len(skeleton_points), 2), dtype=np.float64)
            for i, point in enumerate(skeleton_points):

                py, px = point

                x = (px - self.map_width / 2) * self.map_resolution
                y = (py - self.map_height / 2) * self.map_resolution

                waypoints[i] = [x, y]

            self.waypoint_groups.append(waypoints)

    def find_nearest_leaf_node_along_path(
        self, current_node: int, leaf_nodes: list, graph: nx.Graph
    ) -> int:

        nearest_leaf_node = min(
            leaf_nodes,
            key=lambda x: len(
                nx.shortest_path(graph, source=current_node, target=x)
            ),
        )

        return nearest_leaf_node

    def get_path(self, source, target, graph) -> list:
        return nx.shortest_path(graph, source=source, target=target)

    def plan_path(self, start: np.ndarray,
                  graph: nx.Graph,
                  waypoints: np.ndarray) -> np.ndarray:
        leaf_nodes = self.find_leaf_nodes(graph)
        self.leaf_nodes.extend(leaf_nodes)

        if len(leaf_nodes) == 0:
            raise ValueError("No leaf nodes found")

        visited_leaf_nodes = set()
        current_leaf = self.find_nearest_leaf_node(graph, start, leaf_nodes)
        ordered_nodes = [current_leaf]

        while len(visited_leaf_nodes) < len(leaf_nodes):
            visited_leaf_nodes.add(current_leaf)
            remaining = [n for n in leaf_nodes if n not in visited_leaf_nodes]

            if not remaining:
                break

            next_leaf = self.find_nearest_leaf_node_along_path(current_leaf, remaining, graph)
            path = self.get_path(current_leaf, next_leaf, graph)
            ordered_nodes.extend(path[1:])
            current_leaf = next_leaf
        path = np.array([waypoints[node] for node in ordered_nodes])
        return path

    def generate_path(self, start=None):
        self.read()

        self.paths = []
        for waypoints in self.waypoint_groups:

            if len(waypoints) < 2:
                continue

            graph = self.set_waypoints(waypoints, self.resolution)
            path = self.plan_path(start, graph, waypoints)
            self.paths.append(path)
            start = path[-1]

        if self.node is not None:
            self.publish_path()


class SpanningTreePath(SystematicNavigator):
    name = "SpanningTreePlanner"

    def __init__(self, node=None, resolution=0.1, scale=0.1):
        super().__init__(node, resolution)
        self.scale = scale

    def sample_map(self, map, scale):
        grid = cv2.resize(
            map.astype(np.uint8),
            (0, 0),
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_NEAREST_EXACT,
        )
        return grid

    def subdivide(self, G):
        H = nx.Graph()
        for u, v in G.edges():
            H.add_edge(u, v)
            mid_point = ((u[0] + v[0]) / 2, (u[1] + v[1]) / 2)
            H.add_node(mid_point)
            H.add_edge(u, mid_point)
            H.add_edge(v, mid_point)
        return H

    def spanning_tree(self, grid):
        G = nx.grid_2d_graph(*grid.shape)
        for u, v in G.edges():
            if grid[u] == 0 or grid[v] == 0:
                G.remove_edge(u, v)
        free_nodes = [n for n in G.nodes() if G.degree(n) > 0]

        if not free_nodes:
            raise ValueError("Empty graph")

        start = free_nodes[0]
        self.spanning_tree = nx.DiGraph()
        visited = set()

        for node in G.nodes():
            if node not in visited and G.degree(node) > 0:
                t = nx.dfs_tree(G, source=node)
                self.spanning_tree = nx.compose(self.spanning_tree, t)
                visited.update(t.nodes())
        self.graph = self.subdivide(self.spanning_tree)

    def generate_waypoints(self):
        path_points = list(self.graph.nodes())
        transformed_path_points = (np.array(path_points) + 0.5) / (self.scale)

        self.perimiter_map = np.zeros_like(self.map)
        self.size = int(1 / (self.scale * 4))

        self.tree_points = []
        for p in transformed_path_points:
            x = int(round(p[1]))
            y = int(round(p[0]))
            self.tree_points.append((x, y))
            cv2.rectangle(
                self.perimiter_map,
                (x - self.size, y - self.size),
                (x + self.size, y + self.size),
                255,
                -1,
            )

        self.waypoints, _ = cv2.findContours(
            self.perimiter_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return

    def generate_path(self):
        return


PLANNERS = {
    BousPath.name: BousPath,
    SkeletonPath.name: SkeletonPath,
    SpanningTreePath.name: SpanningTreePath,
    StraightLinePath.name: StraightLinePath,
}
