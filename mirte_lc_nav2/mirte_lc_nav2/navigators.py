import numpy as np
import cv2

import rclpy

from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator
import mirte_lc_nav2.utils as ut
from mirte_lc_nav2.utils import LogType
# from navigator_types import SystematicNavigator, ReactiveNavigator
# import utils as ut
# from utils import LogType

import trajgenpy as tjp
from trajgenpy import Geometries

import shapely
from shapely.validation import explain_validity
from shapely.geometry.polygon import orient

from scipy.spatial import KDTree
import networkx as nx
from skimage.morphology import skeletonize

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

    def bcd(self):
        if self.node is not None:
            ut.log(self.node, LogType.INFO, f"Contours: {len(self.polymap)}")

        for i, c in enumerate(self.polymap):
            ut.log(self.node, LogType.INFO, f"{i}: {len(c)} points")

        polygons = self.polymap.copy()
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

        outer = shapely.Polygon(outer_poly)
        polygons.pop(0)

        holes = []

        for contour in polygons:
            poly = shapely.Polygon(contour).simplify(0.1, preserve_topology=True)

            if not poly.is_valid:
                ut.log(self.node, LogType.WARN, f"Invalid polygon: {explain_validity(poly)}")
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

        ut.log(self.node, LogType.INFO, 'Performing Decomposition')
        polygon_list = Geometries.decompose_polygon(outer, 
                                                    obstacles=obstacles
                                                    )
        self.cells = polygon_list
        self.raw_cells = [
                            np.array(polygon.exterior.coords, dtype=np.int32)
                            for polygon in polygon_list
                        ]

        ut.log(self.node, LogType.INFO, "map decomposed, publishing decomposition")
        if self.node is not None:
            self.publish_decomposition()

    def bous_path(self, robot_width=0.3):
        ut.log(self.node, LogType.INFO, f"Number of cells: {len(self.cells)}")

        offset = Geometries.get_sweep_offset(overlap=0.0, height=0.6, field_of_view=90)
        # result = []
        self.paths = []

        ut.log(self.node, LogType.INFO, "generating full trajectory")
        for cell in self.cells:
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
        self.bcd()
        self.bous_path()


class SkeletonPath(SystematicNavigator):
    name = "SkeletonPlanner"
    def __init__(self, node=None, resolution=0.1):
        super().__init__(node, resolution)

        self.MAP_CELL_LIST_UNKNOWN   = -1
        self.MAP_CELL_LIST_FREE      = 0
        self.MAP_CELL_LIST_OCCUPIED  = 100

        self.MAP_CELL_PGM_UNKNOWN    = 205
        self.MAP_CELL_PGM_FREE       = 254
        self.MAP_CELL_PGM_OCCUPIED   = 0

        self.MAP_SMOOTHING_SIGMA = 4
        self.MAP_CRISP_THRESHOLD = 250
        self.MAP_CONTOUR_SKIP = 20
        self.MAP_GRID_SIZE = 10
        self.MAP_MERGE_DISTANCE = 10

        self.origin = None
        self.graph = None
        self.start = None
        self.path = None
        self.waypoints = None
        self.offset = None

    def read(self, plot=False) -> np.ndarray:
        '''
            Generates a skeleton tree image
            of the same size as self.map.
        '''
        contour_map = np.zeros_like(self.map)
        self.origin = np.array([
            self.map_height / 2,
            self.map_width / 2
        ])        
        self.contour_img = cv2.drawContours(contour_map, [self.contours[0]], -1, (255), -1)
        # Get the skeleton of the area
        skeleton_map = skeletonize(contour_map)
        self.skeleton_map = skeleton_map.astype(np.uint8) * 255
    
        # Convert every point in the skeleton into Cartesian coordinates
        skeleton_points = np.array(np.where(skeleton_map)).T
        self.skeleton_points = skeleton_points
        self.waypoints = np.zeros_like(skeleton_points).astype(np.float64)
        for i, point in enumerate(skeleton_points):
            self.waypoints[i] = (point - self.origin[:2]) * self.map_resolution
        self.waypoints = self.waypoints[:, ::-1]

    def set_waypoints(self, distance=0.0) -> None:
        '''
            Takes skeleton tree image and generates
            a network graph from the tree.
        '''
        waypoints = self.waypoints
        resolution = self.resolution
        # # Set the waypoints and create a tree for the waypoints

        tree = KDTree(waypoints)

        self.graph = nx.Graph()

        for i, p in enumerate(waypoints):
            self.graph.add_node(i, pos=tuple(p))

            indices = tree.query_ball_point(p, resolution * 0.8)

            for j in indices:
                if i == j:
                    continue
                self.graph.add_edge(i, j)

    def find_leaf_nodes(self) -> list:
        leaf_nodes = [node for node in self.graph.nodes if self.graph.degree(node) == 1]
        return leaf_nodes
    
    def find_nearest_leaf_node(self, current_position: np.ndarray, leaf_nodes: list) -> int:
        nearest_leaf_node = min(
            leaf_nodes, key=lambda x: np.linalg.norm(
                current_position - np.array(self.graph.nodes[x]['pos'])
                )
            )
        return nearest_leaf_node

    def find_nearest_leaf_node_along_path(
        self,
        current_node: int,
        leaf_nodes: list
    ) -> int:

        nearest_leaf_node = min(
            leaf_nodes,
            key=lambda x: len(
                nx.shortest_path(
                    self.graph,
                    source=current_node,
                    target=x
                )
            )
        )

        return nearest_leaf_node
    
    def get_path(self, source, target) -> list:
        path = nx.shortest_path(self.graph, source=source, target=target)
        return path               

    def plan_path(self) -> np.ndarray:
        assert self.start is not None, 'The starting position of the robot has not been set'
        assert self.graph is not None, 'The graph has not been created'

        # Instantiate sets of nodes, and leaf nodes
        visited_nodes = set()
        visited_leaf_nodes = set()
        leaf_nodes = self.find_leaf_nodes()
        leaf_node_count = len(leaf_nodes)
        ut.log(
                self.node,
                LogType.INFO,
                f"found {leaf_node_count} leaf nodes",
            )

        # Find the nearest leaf node to the starting position
        starting_leaf_node = self.find_nearest_leaf_node(self.start, leaf_nodes)
        ordered_nodes = [starting_leaf_node]
        # Loop until all leaf nodes have been visited
        while len(visited_leaf_nodes) < leaf_node_count:

            # Update the visited leaf nodes
            visited_leaf_nodes.add(starting_leaf_node)
            if starting_leaf_node in leaf_nodes:
                leaf_nodes.remove(starting_leaf_node)

            # Stop if no more leaf nodes remain
            if not leaf_nodes:
                break

            # Get the nearest leaf node from the current node
            next_leaf_node = self.find_nearest_leaf_node_along_path(
                starting_leaf_node,
                leaf_nodes
            )

            # Get the path from the current node to the next leaf node
            path = self.get_path(starting_leaf_node, next_leaf_node)

            # Iterate over every node in the path
            for node in path[::self.offset]:
                ordered_nodes.append(node)

            # Update the starting leaf node
            starting_leaf_node = next_leaf_node

        # Convert the visited nodes to array (n, 2) format
        self.path = np.array([self.waypoints[node] for node in ordered_nodes])
        return path[::2]

    def generate_path(self):
        self.read()
        self.set_waypoints()
        self.plan_path()


class TreePath(SystematicNavigator):
    name = "TreePlanner"
    def __init__(self, node=None, resolution=0.1):
        super().__init__(resolution)

    def generate_path(self):
        return


PLANNERS = {
    BousPath.name: BousPath,
    SkeletonPath.name: SkeletonPath,
    TreePath.name: TreePath,
    StraightLinePath.name: StraightLinePath,
}