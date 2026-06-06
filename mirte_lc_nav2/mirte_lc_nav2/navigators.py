import numpy as np
import cv2

import rclpy

from mirte_lc_nav2.navigator_types import SystematicNavigator
from mirte_lc_nav2.utils import LogType
import mirte_lc_nav2.utils as ut

# Use these imports when running in Jupyter
# from navigator_types import SystematicNavigator
# import utils as ut
# from utils import LogType

# import trajgenpy as tjp
# from trajgenpy import Geometries

import shapely
from shapely.validation import explain_validity
from shapely.geometry.polygon import orient
from shapely.geometry import Polygon

from scipy.spatial import KDTree
import networkx as nx
from skimage.morphology import (
    skeletonize,
    medial_axis,
    thin,
    max_tree,
    binary_closing,
)
from scipy.interpolate import interp1d


class StraightLinePath(SystematicNavigator):
    """
    Simple systematic planner that generates a straight-line trajectory.

    The planner starts from a given pose and generates evenly spaced
    waypoints along a diagonal line.

    Attributes
    ----------
    name : str
        Planner identifier.
    length : float
        Total length of the generated line in meters.
    start_pose : tuple
        Starting pose in the form (x, y, yaw).
    """

    name = "StraightLinePlanner"

    def __init__(self, node=None, resolution=0.1, length=2.0):
        """
        Initialize the straight-line planner.

        Parameters
        ----------
        node : rclpy.node.Node | None, optional
            ROS2 node used for logging and visualization.
        resolution : float, optional
            Distance between generated waypoints in meters.
        length : float, optional
            Total trajectory length in meters.
        """
        super().__init__(node, resolution)
        self.length = length
        self.start_pose = (0, 0, 0)

    def generate_path(self):
        """
        Generate a straight-line trajectory.

        The path begins at `self.start_pose` and extends diagonally
        with waypoints separated by the configured resolution.

        Returns
        -------
        None
        """
        if self.start_pose is None:
            return None

        x0, y0, yaw = self.start_pose
        n_points = int(self.length / self.path_resolution)

        path = []

        for i in range(n_points):
            d = i * self.path_resolution
            path.append([x0 + d, y0 + d, yaw])

        self.paths = np.array([path])


# class BousPath(SystematicNavigator):
#     """
#     Coverage path planner using boustrophedon cellular decomposition.

#     The planner:
#     1. Decomposes free space into convex cells.
#     2. Generates sweep trajectories for each cell.

#     Attributes
#     ----------
#     name : str
#         Planner identifier.
#     """

#     name = "BousPlanner"

#     def __init__(self, node=None, resolution=0.1):
#         """
#         Initialize the Boustrophedon planner.

#         Parameters
#         ----------
#         node : rclpy.node.Node | None, optional
#             ROS2 node used for logging and visualization.
#         resolution : float, optional
#             Planner waypoint spacing in meters.
#         """
#         self.node = node
#         super().__init__(node, resolution)

#     def bcd(self, polygons):
#         """
#         Perform boustrophedon cellular decomposition.

#         Parameters
#         ----------
#         polygons : list
#             List of polygon contours where:
#             - polygons[0] is the outer boundary
#             - polygons[1:] are holes/obstacles

#         Returns
#         -------
#         list | bool
#             List of decomposed convex cells if successful,
#             otherwise False.
#         """
#         if self.node is not None:
#             ut.log(self.node, LogType.INFO, f"Contours: {len(polygons)}")

#         for i, c in enumerate(polygons):
#             ut.log(self.node, LogType.INFO, f"{i}: {len(c)} points")

#         outer_poly = shapely.Polygon(polygons[0])

#         if not outer_poly.is_valid:
#             ut.log(
#                 self.node,
#                 LogType.WARN,
#                 f"Invalid outer polygon: {explain_validity(outer_poly)}",
#             )

#             outer_poly = outer_poly.buffer(0)

#             if not outer_poly.is_valid:
#                 ut.log(self.node, LogType.WARN, "Could not fix outer polygon")
#                 return False

#             if outer_poly.is_empty:
#                 ut.log(self.node, LogType.ERR, "Outer polygon is empty after fix")
#                 return False

#             if outer_poly.geom_type == "MultiPolygon":
#                 ut.log(
#                     self.node,
#                     LogType.WARN,
#                     "Outer polygon became MultiPolygon, taking largest piece",
#                 )

#                 outer_poly = max(outer_poly.geoms, key=lambda p: p.area)

#         outer = outer_poly
#         holes_contours = polygons[1:]

#         holes = []

#         for contour in holes_contours:
#             poly = shapely.Polygon(contour).simplify(
#                 0.1,
#                 preserve_topology=True,
#             )

#             # Check validity and fix
#             if not poly.is_valid:
#                 ut.log(
#                     self.node,
#                     LogType.WARN,
#                     f"Invalid polygon: {explain_validity(poly)}",
#                 )

#                 poly = poly.buffer(0)

#                 if not poly.is_valid:
#                     ut.log(self.node, LogType.WARN, "Could not fix polygon")
#                     continue

#                 if poly.is_empty:
#                     ut.log(self.node, LogType.ERR, "Polygon is empty after fix")
#                     return False

#                 if poly.geom_type == "MultiPolygon":
#                     ut.log(
#                         self.node,
#                         LogType.WARN,
#                         "Polygon became MultiPolygon, taking largest piece",
#                     )

#                     poly = max(poly.geoms, key=lambda p: p.area)

#             holes.append(poly)

#         obstacles = shapely.MultiPolygon(holes)

#         ut.log(self.node, LogType.INFO, "Performing Decomposition")

#         polygon_list = Geometries.decompose_polygon(
#             outer,
#             obstacles=obstacles,
#         )

#         self.raw_cells = [
#             np.array(polygon.exterior.coords, dtype=np.int32)
#             for polygon in polygon_list
#         ]

#         ut.log(
#             self.node,
#             LogType.INFO,
#             "map decomposed, publishing decomposition",
#         )

#         if self.node is not None:
#             self.publish_decomposition()

#         return polygon_list

#     def bous_path(self, cells, robot_width=0.6):
#         """
#         Generate sweep coverage trajectories for decomposed cells.

#         Parameters
#         ----------
#         cells : list
#             List of decomposed polygons.
#         robot_width : float, optional
#             Width of the robot in meters.

#         Returns
#         -------
#         list
#             Generated sweep trajectories.
#         """
#         ut.log(self.node, LogType.INFO, f"Number of cells: {len(cells)}")

#         offset = Geometries.get_sweep_offset(
#             overlap=0.0,
#             height=robot_width,
#             field_of_view=90,
#         )

#         ut.log(self.node, LogType.INFO, "generating full trajectory")

#         paths = []
#         for cell in cells:
#             # cell = self.force_ccw(cell)
#             # cell = orient(cell, 1)
#             print(cell.geom_type)
#             print(cell.exterior.is_ccw)
#             print(cell.area)
#             cell = self.flip_y(cell)
#             sweeps = Geometries.generate_sweep_pattern(
#                 cell,
#                 offset,
#                 clockwise=False,
#                 connect_sweeps=True,
#             )

#             paths_mls = Geometries.GeoMultiTrajectory(sweeps).get_geometry()

#             paths.append(self.multiline_to_coords(paths_mls))

#         if self.node is not None:
#             self.publish_path()

#         return paths

#     def multiline_to_coords(self, multiline):
#         """
#         Convert a MultiLineString trajectory into coordinate lists.

#         Parameters
#         ----------
#         multiline : shapely.MultiLineString | iterable
#             Geometry containing multiple line segments.

#         Returns
#         -------
#         list
#             Flattened list of trajectory coordinates.
#         """
#         coords = []

#         if isinstance(multiline, shapely.MultiLineString):
#             for line in multiline.geoms:
#                 coords.extend(list(line.coords))
#         else:
#             for line in multiline:
#                 coords.extend(list(line.coords))

#         return coords

#     def flip_y(self, poly):
#         return Polygon(
#             [(x, -y) for x, y in poly.exterior.coords],
#             [[(x, -y) for x, y in ring.coords] for ring in poly.interiors]
#         )

#     def generate_path(self, start=None):
#         """
#         Generate complete coverage trajectories for the current map.

#         Parameters
#         ----------
#         start : np.ndarray | None, optional
#             Starting robot position.

#         Returns
#         -------
#         None
#         """
#         self.paths = []
#         self.cells = []

#         for group in self.polymap:
#             cells = self.bcd(group)
#             self.cells.extend(cells)
#             paths = self.bous_path(cells)
#             self.paths.extend(paths)


class SkeletonPath(SystematicNavigator):
    """
    Coverage planner based on skeletonization.

    The planner:
    1. Extracts the medial axis skeleton from free space.
    2. Converts the skeleton into a graph.
    3. Traverses the graph between leaf nodes.

    Attributes
    ----------
    name : str
        Planner identifier.
    """

    name = "SkeletonPlanner"

    def __init__(self, node=None, resolution=0.1):
        """
        Initialize the skeleton planner.

        Parameters
        ----------
        node : rclpy.node.Node | None, optional
            ROS2 node instance.
        resolution : float, optional
            Waypoint spacing in meters.
        """
        super().__init__(node, resolution)

        self.path = None
        self.leaf_nodes = []
        self.offset = 1

    def read(self, plot=False) -> np.ndarray:
        """
        Generate skeleton waypoints from polygon groups.

        This function:
        - Rasterizes polygon contours
        - Computes the medial axis skeleton
        - Converts skeleton pixels to world coordinates

        Parameters
        ----------
        plot : bool, optional
            Unused visualization flag.

        Returns
        -------
        np.ndarray
            Extracted waypoint groups.
        """
        ut.log(self.node, LogType.INFO, "reading map")

        self.waypoint_groups = []

        self.contour_map = np.zeros_like(
            self.map,
            dtype=np.uint8,
        )

        for group in self.polymap:
            contour_map = np.zeros_like(
                self.map,
                dtype=np.uint8,
            )

            for cont in group:
                contour = np.asarray(self.world_to_pixel_poly(cont), dtype=np.float32)

                contour = contour.reshape((-1, 1, 2)).astype(np.int32)

                self.contour_map = cv2.drawContours(
                    self.contour_map,
                    [contour],
                    -1,
                    255,
                    -1,
                )

                local_contour_map = cv2.drawContours(
                    contour_map,
                    [contour],
                    -1,
                    255,
                    -1,
                )

            # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            # smoothed_contour_map = cv2.morphologyEx(
            #     local_contour_map,
            #     cv2.MORPH_CLOSE,
            #     kernel,
            # )
            
            # # Crop the map to prevent skeleton from going outside bounds
            # crop_size = 20
            # h, w = local_contour_map.shape
            # cropped_map = local_contour_map[crop_size:h-crop_size, crop_size:w-crop_size]
            
            skeleton_map = skeletonize(
                self.binary_costmap > 0,
                # return_distance=False,
            )

            self.skeleton_map = skeleton_map.astype(np.uint8) * 255
            rows, cols = np.where(skeleton_map)
            # Add back the crop offset to get original map coordinates
            skeleton_points = np.column_stack((cols, rows))
            skeleton_points = self.pixel_to_world_poly(skeleton_points)

            waypoints = np.asarray(
                skeleton_points,
                dtype=np.float64,
            )

            self.waypoint_groups.append(waypoints)

        ut.log(self.node, LogType.INFO, "map read")

    def find_nearest_leaf_node_along_path(
        self,
        current_node: int,
        leaf_nodes: list,
        graph: nx.Graph,
    ) -> int:
        """
        Find the nearest leaf node using graph distance.

        Parameters
        ----------
        current_node : int
            Current graph node.
        leaf_nodes : list
            Candidate leaf nodes.
        graph : nx.Graph
            Navigation graph.

        Returns
        -------
        int
            Nearest leaf node index.
        """
        nearest_leaf_node = min(
            leaf_nodes,
            key=lambda x: len(
                nx.shortest_path(
                    graph,
                    source=current_node,
                    target=x,
                )
            ),
        )

        return nearest_leaf_node

    def get_path(self, source, target, graph) -> list:
        """
        Compute the shortest path between two graph nodes.

        Parameters
        ----------
        source : int
            Start node.
        target : int
            Goal node.
        graph : nx.Graph
            Navigation graph.

        Returns
        -------
        list
            Ordered node indices.
        """
        return nx.shortest_path(
            graph,
            source=source,
            target=target,
        )

    def plan_path(
        self,
        start: np.ndarray,
        graph: nx.Graph,
        waypoints: np.ndarray,
    ) -> np.ndarray:
        """
        Generate a traversal path over the skeleton graph.

        Parameters
        ----------
        start : np.ndarray
            Robot start position.
        graph : nx.Graph
            Skeleton connectivity graph.
        waypoints : np.ndarray
            Waypoint coordinates.

        Returns
        -------
        np.ndarray
            Ordered path coordinates.
        """
        leaf_nodes = self.find_leaf_nodes(graph)

        self.leaf_nodes.extend(leaf_nodes)

        if len(leaf_nodes) == 0:
            raise ValueError("No leaf nodes found")

        visited_leaf_nodes = set()

        current_leaf = self.find_nearest_node(
            graph,
            start,
            leaf_nodes,
        )

        ordered_nodes = [current_leaf]

        while len(visited_leaf_nodes) < len(leaf_nodes):
            visited_leaf_nodes.add(current_leaf)

            remaining = [n for n in leaf_nodes if n not in visited_leaf_nodes]

            if not remaining:
                break

            next_leaf = self.find_nearest_leaf_node_along_path(
                current_leaf,
                remaining,
                graph,
            )

            path = self.get_path(
                current_leaf,
                next_leaf,
                graph,
            )

            ordered_nodes.extend(path[1:])

            current_leaf = next_leaf

        path = np.array([waypoints[node] for node in ordered_nodes])

        ut.log(self.node, LogType.INFO, "planned path successfully")

        return path

    def generate_path(self, start=None):
        """
        Generate skeleton-based coverage trajectories.

        Parameters
        ----------
        start : np.ndarray | None, optional
            Starting robot position.

        Returns
        -------
        None
        """
        ut.log(self.node, LogType.INFO, f"generating {self.name} path")
        self.read()
        self.paths = []

        for waypoints in self.waypoint_groups:

            if len(waypoints) < 2:
                continue

            graph = self.set_waypoints(
                waypoints
            )

            path = self.plan_path(
                start,
                graph,
                waypoints,
            )

            self.paths.append(path)

            start = path[-1]

        if self.node is not None:
            self.publish_path()


class SpanningTreePath(SystematicNavigator):
    """
    Coverage planner based on spanning-tree traversal.

    The planner:
    1. Downsamples the occupancy map.
    2. Builds a spanning tree over free cells.
    3. Generates traversal contours.

    Attributes
    ----------
    name : str
        Planner identifier.
    """

    name = "SpanningTreePlanner"

    def __init__(self, node=None, resolution=0.1, scale=0.06):
        """
        Initialize the spanning-tree planner.

        Parameters
        ----------
        node : rclpy.node.Node | None, optional
            ROS2 node instance.
        resolution : float, optional
            Waypoint spacing in meters.
        scale : float, optional
            Downsampling scale factor.
        """
        super().__init__(node, resolution)

        self.scale = scale

    def sample_map(self):
        """
        Downsample an occupancy map.

        Parameters
        ----------
        map : np.ndarray
            Occupancy map.
        scale : float
            Scaling factor.

        Returns
        -------
        np.ndarray
            Downsampled occupancy grid.
        """
        grid = cv2.resize(
            self.binary_costmap.astype(np.uint8),
            (0, 0),
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_NEAREST_EXACT,
        )

        return grid

    def subdivide(self, G):
        """
        Subdivide graph edges by inserting midpoint nodes.

        Parameters
        ----------
        G : nx.Graph
            Input graph.

        Returns
        -------
        nx.Graph
            Subdivided graph.
        """
        H = nx.Graph()

        for u, v in G.edges():
            H.add_edge(u, v)

            mid_point = (
                (u[0] + v[0]) / 2,
                (u[1] + v[1]) / 2,
            )

            H.add_node(mid_point)

            H.add_edge(u, mid_point)
            H.add_edge(v, mid_point)

        return H

    def spanning_tree(self, grid):
        """
        Generate a spanning tree over free-space cells.

        Parameters
        ----------
        grid : np.ndarray
            Binary occupancy grid.

        Returns
        -------
        None
        """
        G = nx.grid_2d_graph(*grid.shape)

        for u, v in G.edges():
            if grid[u] == 0 or grid[v] == 0:
                G.remove_edge(u, v)

        free_nodes = [n for n in G.nodes() if G.degree(n) > 0]

        if not free_nodes:
            raise ValueError("Empty graph")

        self.spanning_tree_graph = nx.DiGraph()

        visited = set()

        for node in G.nodes():
            if node not in visited and G.degree(node) > 0:
                t = nx.dfs_tree(G, source=node)

                self.spanning_tree_graph = nx.compose(
                    self.spanning_tree_graph,
                    t,
                )

                visited.update(t.nodes())

        self.graph = self.subdivide(self.spanning_tree_graph)

    def generate_waypoint_contours(self):
        """
        Generate contour regions around spanning-tree paths for circumnavigation.

        Returns
        -------
        None
        """
        path_points = list(self.graph.nodes())

        transformed_path_points = (np.array(path_points) + 0.5) / self.scale

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

        perimeter_u8 = self.perimiter_map.astype(np.uint8)
        
        self.spanning_contours, _ = cv2.findContours(
            perimeter_u8,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        return
    
    def read(self):
        self.waypoint_groups = []
        
        grid = self.sample_map()
        self.spanning_tree(grid)
        self.generate_waypoint_contours()
        
        for contour in self.spanning_contours:
            waypoints = self.get_waypoints(contour)
            self.waypoint_groups.append(waypoints)
        

    def get_waypoints(self, contour):
        """
        Convert contour pixels into world-coordinate waypoints.
        """

        contour = contour.squeeze()

        if len(contour.shape) != 2:
            return np.empty((0, 2))

        waypoints = self.pixel_to_world_poly(contour)

        return np.asarray(waypoints, dtype=np.float64)

    def plan_path(
        self,
        start: np.ndarray,
        graph: nx.Graph,
        waypoints: np.ndarray,
    ) -> np.ndarray:
        """
        Resample contour into evenly spaced path points.
        """

        # # Check the start and remove the edge right after
        # closest_node = self.find_nearest_node(graph, start, list(graph.nodes()))
        # neighbors = list(graph.neighbors(closest_node))

        # if neighbors:
        #     graph.remove_edge(
        #         closest_node,
        #         neighbors[0]
        #     )

        # graph_path = nx.shortest_path(
        #     graph,
        #     closest_node,
        #     neighbors[0])

        # path = np.array([waypoints[node] for node in graph_path])

        idx = np.argmin(
            np.linalg.norm(
                waypoints - start,
                axis=1
            )
        )

        path = np.concatenate(
            [
                waypoints[idx:],
                waypoints[:idx]
            ]
        )

        ut.log(self.node, LogType.INFO, "planned path successfully")

        return path

    def generate_path(self, start=None):
        """
        Generate coverage trajectories using spanning trees.

        Parameters
        ----------
        start : np.ndarray | None, optional
            Starting robot position.

        Returns
        -------
        None
        """
        ut.log(self.node, LogType.INFO, f"generating {self.name} path")
        self.read()
        self.paths = []

        for waypoints in self.waypoint_groups:
            
            if len(waypoints) < 2:
                continue
            
            graph = self.set_waypoints(
                waypoints
            )

            path = self.plan_path(
                start,
                graph,
                waypoints,
            )
            
            self.paths.append(path)
            
            start = path[-1]

        if self.node is not None:
            self.publish_path()


PLANNERS = {
    # BousPath.name: BousPath,
    SkeletonPath.name: SkeletonPath,
    SpanningTreePath.name: SpanningTreePath,
    StraightLinePath.name: StraightLinePath,
}
