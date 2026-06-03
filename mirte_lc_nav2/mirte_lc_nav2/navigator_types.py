import numpy as np
import cv2
import os

from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

# Use these imports when running in Jupyter
# import utils as ut
# from utils import  LogType

import mirte_lc_nav2.utils as ut
from mirte_lc_nav2.utils import LogType

import networkx as nx
from scipy.spatial import KDTree

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_simple_commander.costmap_2d import PyCostmap2D
import rclpy


class SystematicNavigator(BasicNavigator):
    """
    Base class for systematic coverage path planners.

    This class provides shared functionality for:
    - Occupancy map preprocessing
    - Contour extraction
    - Coordinate conversion
    - ROS visualization publishing
    - Debug visualization

    Child classes should implement their own `generate_path()`
    method to produce coverage trajectories.

    Attributes
    ----------
    navigator_type : str
        Identifier for the planner category.
    paths : list | None
        Generated coverage paths.
    map : np.ndarray | None
        Occupancy grid map.
    polymap : list | None
        Polygon representation of the map contours.
    threshold : float
        Occupancy threshold used to classify obstacles.
    resolution : float
        Planner waypoint resolution in meters.
    map_resolution : float
        Resolution of the occupancy map in meters/pixel.
    node : rclpy.node.Node | None
        ROS2 node used for logging and publishers.
    """

    def __init__(
        self, node=None, resolution=0.1, lethal_threshold=20.0
    ):
        """
        Initialize the systematic navigator.

        Parameters
        ----------
        node : rclpy.node.Node | None, optional
            ROS2 node instance.
        resolution : float, optional
            Desired waypoint spacing in meters.
        map_resolution : float, optional
            Occupancy map resolution in meters/pixel.
        lethal_threshold : float, optional
            Threshold above which cells are considered occupied.
        """
        super().__init__(node_name="coverage_navigator")
        self.navigator_type = "systematic"
        self.paths = None
        self.map = None
        self.polymap = None
        self.threshold = lethal_threshold
        self.path_resolution = resolution
        self.node = node
        self.graph = None
        self.start = None
        self.waypoints = None

        if self.node is not None:
            self.polymap_publisher = self.node.create_publisher(
                PolygonStamped, "/systematic_navigator/map_contours", 10
            )

            self.decomp_publisher = self.node.create_publisher(
                MarkerArray, "/systematic_navigator/decomposed_map", 10
            )

            self.path_publisher = self.node.create_publisher(
                MarkerArray, "/systematic_navigator/planned_path", 10
            )

    def plan(self, map_msg, start: np.ndarray = np.zeros(2), show=False) -> None:
        """
        Generate a coverage plan for a given occupancy map.

        Parameters
        ----------
        new_map : np.ndarray
            Occupancy grid map.
        start : np.ndarray, optional
            Starting robot position in world coordinates.
        show : bool, optional
            Unused visualization flag.

        Returns
        -------
        None
        """
        self.start = start
        ut.log(self.node, LogType.INFO, "updating map")
        self.update_map(map_msg)
        self.generate_path(start)
        self.sanitize_paths()

    def sanitize_paths(self):
        """Clamp generated paths to the valid occupancy grid bounds."""
        if self.paths is None:
            return

        if not hasattr(self, "costmap") or self.costmap is None:
            return

        ox = self.costmap.origin_x
        oy = self.costmap.origin_y
        res = self.costmap.resolution
        min_x = ox + 0.5 * res
        min_y = oy + 0.5 * res
        max_x = ox + (self.costmap.size_x - 0.5) * res
        max_y = oy + (self.costmap.size_y - 0.5) * res

        def clamp_point(p):
            x = min(max_x, max(min_x, float(p[0])))
            y = min(max_y, max(min_y, float(p[1])))
            if len(p) > 2:
                return [x, y, float(p[2])]
            return [x, y]

        sanitized = []
        for segment in self.paths:
            if segment is None:
                continue
            sanitized_segment = [clamp_point(p) for p in segment]
            # Remove duplicates introduced by clamping
            filtered = []
            for p in sanitized_segment:
                if not filtered or p[0] != filtered[-1][0] or p[1] != filtered[-1][1]:
                    filtered.append(p)
            sanitized.append(np.asarray(filtered, dtype=np.float64))

        self.paths = sanitized

    def update_map(self, map_msg):
        """
        Convert an occupancy map into polygon contours.

        This function:
        - Thresholds the occupancy map
        - Extracts contours
        - Converts contours to world coordinates
        - Stores the polygon map representation

        Parameters
        ----------
        new_map : np.ndarray
            Occupancy grid map.

        Returns
        -------
        None
        """
        # Set up Costmap Object
        self.costmap = PyCostmap2D(map_msg)
        self.map = self.costmap.costmap
        self.map = np.array(self.map, dtype=np.int16).reshape((self.costmap.size_x, self.costmap.size_y))
        
        self.binary_costmap = np.zeros_like(self.map, dtype=np.uint8)
        self.binary_costmap[self.map < self.threshold] = 255
        self.binary_costmap[self.map == -1] = 0

        margin_m = 0.3
        margin_px = max(1, int(margin_m / self.costmap.resolution))
        
        if margin_px % 2 == 0:
            margin_px += 1

        # Find Contours
        contours, _ = cv2.findContours(
            self.binary_costmap, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = [
            cv2.approxPolyDP(c, 0.005 * cv2.arcLength(c, True), True) for c in contours
        ]
        
        # Sort Contours from largest to smallest
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        outer_contour = contours[0]

        # Filter out contours that lie outside the largest contour
        filtered_contours = [outer_contour]

        for contour in contours[1:]:  # skip outer
            if len(contour) < 4:
                ut.log(
                    self.node,
                    LogType.WARN,
                    f"Skipping contour with {len(contour)} points",
                )
                continue
            filtered_contours.append(contour)

        self.contours = filtered_contours

        closed_contour = []

        for contour in filtered_contours:
            pts = contour[:, 0, :]

            points = self.pixel_to_world_poly(pts)

            if points[0] != points[-1]:
                points.append(points[0])

            closed_contour.append(points)

        if self.node is not None:
            self.publish_polygon(closed_contour)

        self.polymap = self.find_polygon_groups(closed_contour)
        return None

    def world_to_pixel_poly(self, polygon):
        return [
            [
                max(0, min(self.costmap.size_x - 1, self.costmap.worldToMap(vertex[0], vertex[1])[0])),
                max(0, min(self.costmap.size_y - 1, self.costmap.worldToMap(vertex[0], vertex[1])[1])),
            ]
            for vertex in polygon
        ]
    
    def pixel_to_world_poly(self, polygon):
        return [
            list(
                self.costmap.mapToWorld(
                    int(np.clip(vertex[0], 0, self.costmap.size_x - 1)),
                    int(np.clip(vertex[1], 0, self.costmap.size_y - 1)),
                )
            )
            for vertex in polygon
        ]

    def is_inside(self, contour, outer_contour):
        """
        Check whether a contour lies completely inside another contour.

        Parameters
        ----------
        contour : np.ndarray
            Contour to test.
        outer_contour : np.ndarray
            Reference contour.

        Returns
        -------
        bool
            True if the contour lies inside the outer contour,
            otherwise False.
        """
        ut.log(
            self.node,
            LogType.DEBUG,
            f"Testing if contour {contour} points is inside contour {outer_contour} points",
        )
        for pt in contour:
            if (
                cv2.pointPolygonTest(
                    np.array(outer_contour, dtype=np.float32),
                    (float(pt[0]), float(pt[1])),
                    False,
                )
                < 0
            ):
                return False
        return True

    def find_polygon_groups(self, polymap: np.ndarray) -> list:
        """
        Group contours into polygons with holes.
        """

        sorted_polymap = sorted(polymap, key=polygon_area, reverse=True)

        if len(sorted_polymap) == 0:
            return []

        grouped_polygons = []

        remaining = sorted_polymap.copy()

        ut.log(self.node, LogType.INFO, "sorted polygons, moving on to group detection")

        while remaining:

            outer = remaining.pop(0)
            group = [outer]
            still_remaining = []

            for contour in remaining:
                if self.is_inside(contour, outer):
                    group.append(contour)
                else:
                    still_remaining.append(contour)

            grouped_polygons.append(sorted(group, key=polygon_area, reverse=True))
            remaining = still_remaining

        ut.log(self.node, LogType.INFO, "successfully grouped polygons")

        return grouped_polygons

    def set_waypoints(self, waypoints) -> nx.Graph:
        """
        generates a network graph from waypoints
        """
        # Set the waypoints and create a tree for the waypoints

        tree = KDTree(waypoints)

        graph = nx.Graph()

        for i, p in enumerate(waypoints):
            graph.add_node(i, pos=tuple(p))

            indices = tree.query_ball_point(p, self.path_resolution * 0.8)

            for j in indices:
                if i == j:
                    continue
                graph.add_edge(i, j)

        # Connect any disconnected components in this group
        while nx.number_connected_components(graph) > 1:

            components = list(nx.connected_components(graph))

            # take two different components
            c1 = components[0]
            c2 = components[1]

            best_pair = None
            best_dist = float("inf")

            # find closest pair between components
            for i in c1:
                for j in c2:
                    dist = np.linalg.norm(waypoints[i] - waypoints[j])
                    if dist < best_dist:
                        best_dist = dist
                        best_pair = (i, j)

            # connect them
            i, j = best_pair
            graph.add_edge(i, j)

        return graph

    def find_leaf_nodes(self, graph) -> list:
        leaf_nodes = [node for node in graph.nodes if graph.degree(node) == 1]
        return leaf_nodes

    def find_nearest_node(
        self, graph, current_position: np.ndarray, nodes: list
    ) -> int:
        nearest_leaf_node = min(
            nodes,
            key=lambda x: np.linalg.norm(
                current_position - np.array(graph.nodes[x]["pos"])
            ),
        )
        return nearest_leaf_node

    def publish_polygon(self, contours):
        """
        Publish polygon contours as a ROS PolygonStamped message.

        Parameters
        ----------
        contours : list
            List of contour coordinate lists.

        Returns
        -------
        None
        """
        msg = PolygonStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()

        for contour in contours:
            for px, py in contour:
                point = Point32()
                point.x = px
                point.y = py
                point.z = 0.0
                msg.polygon.points.append(point)

        self.polymap_publisher.publish(msg)

    def publish_decomposition(self):
        """
        Publish decomposed coverage cells as RViz markers.

        Each decomposition cell is visualized as a colored line strip.

        Returns
        -------
        None
        """
        marker_array = MarkerArray()

        for i, cell in enumerate(self.cells):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()

            marker.ns = "decomposition"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.02

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            for p in cell.exterior.coords:
                pt = PoseStamped().pose.position
                pt.x = p[0]
                pt.y = p[1]
                pt.z = 0.0
                marker.points.append(pt)

            marker_array.markers.append(marker)

        self.decomp_publisher.publish(marker_array)

    def publish_path(self):
        """
        Publish planned coverage paths as RViz markers.

        Returns
        -------
        None
        """
        marker_array = MarkerArray()

        for waypoints in self.waypoint_groups:
            id = 0
            for p in waypoints:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.node.get_clock().now().to_msg()

                marker.ns = "path"
                marker.id = id
                id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02

                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0

                marker.pose.position.x = p[0]
                marker.pose.position.y = p[1]
                marker.pose.position.z = 0.0
                marker_array.markers.append(marker)
        self.path_publisher.publish(marker_array)

    def debug_save(self, contours):
        """
        Save a debug visualization of the occupancy map and contours.

        Parameters
        ----------
        contours : list
            Contours to overlay on the map image.

        Returns
        -------
        None
        """
        img = cv2.normalize(self.map.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX)
        img = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

        path = os.path.join(os.path.dirname(__file__), "map.png")
        cv2.imwrite(path, img)

    def publish_info(self):
        """
        Publish all available planner visualization data.

        Publishes:
        - Polygon contours
        - Decomposition cells
        - Planned paths

        Returns
        -------
        None
        """
        if self.poly_map:
            self.polymap_publisher.publish()

        if self.decomp_map:
            self.decomp_publisher.publish()

        if self.paths:
            self.paths_publisher.publish()


def polygon_area(poly):
    x = np.array([p[0] for p in poly])
    y = np.array([p[1] for p in poly])
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))