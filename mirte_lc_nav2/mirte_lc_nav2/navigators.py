import numpy as np
import cv2

from mirte_lc_nav2.navigator_types import SystematicNavigator
from mirte_lc_nav2.utils import LogType
import mirte_lc_nav2.utils as ut

# Use these imports when running in Jupyter
# from navigator_types import SystematicNavigator
# import utils as ut
# from utils import LogType

import networkx as nx
from skimage.morphology import (
    skeletonize,
    medial_axis,
    thin,
)
from scipy.spatial import Voronoi


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
            
            skeleton_map = skeletonize(
                self.binary_costmap > 0,
            )

            self.skeleton_map = skeleton_map.astype(np.uint8) * 255
            rows, cols = np.where(skeleton_map)

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


class CVTPath(SystematicNavigator):
    name="CVTPlanner"
    
    def __init__(self, node=None, resolution=0.1, area=0.06):
        super().__init__(node, resolution)
        self.area = area
    
    def generate_path(self):
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

    def read(self):
        ...

    def get_free_pixels(self):
        """Return pixel coordinates of all free cells in the costmap."""
        rows, cols = np.where(self.binary_costmap > 0)
        return np.column_stack((cols, rows)).astype(np.float64)  # (x, y)

    def plam_path(self):
        ...

class CVTPath(SystematicNavigator):
    """
    Waypoint planner based on Centroidal Voronoi Tessellation (CVT).

    The planner:
    1. Samples N seed points in free space.
    2. Iterates Lloyd's algorithm to converge to CVT centroids.
    3. Solves a nearest-neighbor TSP over the centroids.

    Reference: Cortes, Martinez, Karatas & Bullo,
    "Coverage Control for Mobile Sensing Networks",
    IEEE Trans. Robotics and Automation, 2004.

    Attributes
    ----------
    name : str
        Planner identifier.
    n_seeds : int
        Number of Voronoi cells (waypoints).
    n_iterations : int
        Number of Lloyd iterations.
    """

    name = "CVTPlanner"

    def __init__(self, node=None, resolution=0.1, n_seeds=30, n_iterations=20):
        super().__init__(node, resolution)
        self.n_seeds = n_seeds
        self.n_iterations = n_iterations

    def get_free_pixels(self):
        """Return pixel coordinates of all free cells in the costmap."""
        rows, cols = np.where(self.binary_costmap > 0)
        return np.column_stack((cols, rows)).astype(np.float64)  # (x, y)

    def lloyd_iteration(self, seeds, free_pixels):
        """
        Run one Lloyd iteration: assign pixels to nearest seed,
        then move each seed to the centroid of its cell.

        Parameters
        ----------
        seeds : np.ndarray, shape (N, 2)
        free_pixels : np.ndarray, shape (M, 2)

        Returns
        -------
        np.ndarray
            Updated seed positions.
        """
        tree = KDTree(seeds)
        _, labels = tree.query(free_pixels)

        new_seeds = np.zeros_like(seeds)
        for i in range(len(seeds)):
            members = free_pixels[labels == i]
            if len(members) > 0:
                new_seeds[i] = members.mean(axis=0)
            else:
                new_seeds[i] = seeds[i]  # keep seed if cell is empty

        return new_seeds

    def tsp_nearest_neighbor(self, points, start_idx=0):
        """
        Solve TSP with a nearest-neighbor heuristic.

        Parameters
        ----------
        points : np.ndarray, shape (N, 2)
        start_idx : int

        Returns
        -------
        np.ndarray
            Points in visit order.
        """
        unvisited = list(range(len(points)))
        order = [start_idx]
        unvisited.remove(start_idx)

        while unvisited:
            current = order[-1]
            dists = np.linalg.norm(points[unvisited] - points[current], axis=1)
            nearest = unvisited[int(np.argmin(dists))]
            order.append(nearest)
            unvisited.remove(nearest)

        return points[order]

    def generate_path(self, start=None):
        ut.log(self.node, LogType.INFO, f"generating {self.name} path")

        free_pixels = self.get_free_pixels()

        if len(free_pixels) < self.n_seeds:
            ut.log(self.node, LogType.WARN, "Fewer free pixels than seeds, reducing n_seeds")
            self.n_seeds = len(free_pixels)

        # 1. Initialise seeds randomly from free pixels
        idx = np.random.choice(len(free_pixels), self.n_seeds, replace=False)
        seeds = free_pixels[idx].astype(np.float64)

        # 2. Lloyd iterations
        for _ in range(self.n_iterations):
            seeds = self.lloyd_iteration(seeds, free_pixels)

        # 3. Keep only seeds that landed in free space
        tree = KDTree(free_pixels)
        dists, _ = tree.query(seeds)
        seeds = seeds[dists < 3.0]  # tolerance in pixels

        # 4. Convert pixel centroids to world coordinates
        waypoints = np.array(self.pixel_to_world_poly(seeds.astype(np.int32)))

        # 5. Find nearest seed to start and solve TSP from there
        if start is not None:
            dists_to_start = np.linalg.norm(waypoints - np.array(start[:2]), axis=1)
            start_idx = int(np.argmin(dists_to_start))
        else:
            start_idx = 0

        path = self.tsp_nearest_neighbor(waypoints, start_idx=start_idx)

        self.paths = [path]

        if self.node is not None:
            self.publish_path()

PLANNERS = {
    SkeletonPath.name: SkeletonPath,
    SpanningTreePath.name: SpanningTreePath,
    StraightLinePath.name: StraightLinePath,
}
