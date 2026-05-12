import numpy as np
import cv2
import os

from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# import utils as ut
# from utils import  LogType

import mirte_lc_nav2.utils as ut
from mirte_lc_nav2.utils import  LogType

class SystematicNavigator():
    def __init__(self, node=None, resolution=0.1, map_resolution=0.05, lethal_threshold=20.0):
        self.navigator_type = 'systematic'
        self.paths = None
        self.map = None
        self.polymap = None
        self.threshold = lethal_threshold
        self.resolution = resolution
        self.map_resolution = map_resolution
        self.node = node

        if self.node is not None:
            self.polymap_publisher = self.node.create_publisher(
                PolygonStamped, '/systematic_navigator/map_contours', 10
                )

            self.decomp_publisher = self.node.create_publisher(
                MarkerArray, '/systematic_navigator/decomposed_map', 10
                )

            self.path_publisher = self.node.create_publisher(
                MarkerArray, '/systematic_navigator/planned_path', 10
                )

    def plan(self, new_map, start:np.ndarray = np.zeros(2), show=False) -> None:
        self.start = start
        self.update_map(new_map)
        self.generate_path()

    def update_map(self, new_map):
        self.map = new_map

        self.binary_costmap = np.zeros_like(self.map, dtype=np.uint8)
        self.binary_costmap[self.map < self.threshold] = 255
        self.binary_costmap[self.map == -1] = 0

        margin_m = 0.3
        margin_px = max(1, int(margin_m / self.map_resolution))

        if margin_px % 2 == 0:
            margin_px += 1

        contours, _ = cv2.findContours(self.binary_costmap, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cv2.approxPolyDP(c, 0.005 * cv2.arcLength(c, True), True) for c in contours]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        outer_contour = contours[0]

        # Filter out contours that lie outside the largest contour
        filtered_contours = [outer_contour]

        for contour in contours[1:]:  # skip outer
            if len(contour) < 4:
                ut.log(self.node, LogType.WARN, f"Skipping contour with {len(contour)} points")
                continue
            filtered_contours.append(contour)

        self.contours = filtered_contours
        
        closed_contour = []
        for contour in filtered_contours:
            pts = contour[:, 0, :]

            self.map_height = self.map.shape[0]
            self.map_width = self.map.shape[1]

            points = [
                ((px - 0.5 * self.map_width) * self.map_resolution,
                 (py - 0.5 * self.map_height) * self.map_resolution)
                for px, py in pts
            ]

            if points[0] != points[-1]:
                points.append(points[0])
            
            closed_contour.append(points)

        if self.node is not None:
            self.publish_polygon(closed_contour)

        self.polymap = closed_contour
        return None

    def is_inside(self, contour, outer_contour):
        for pt in contour[:, 0, :]:
            # returns >0 inside, 0 on edge, <0 outside
            if cv2.pointPolygonTest(outer_contour, (float(pt[0]), float(pt[1])), False) < 0:
                return False
        return True

    def world_to_pixel(self, polygon):
        map_h, map_w = self.map.shape[:2]

        coords = []
        for coord in polygon:
            px = int(coord[0] / self.map_resolution + 0.5 * map_w)
            py = int(coord[1] / self.map_resolution + 0.5 * map_h)
            coords.append([px, py])

        return np.array(coords, dtype=np.int32)

    def world_to_pixel_path(self, path):
        map_h, map_w = self.map.shape[:2]

        pts = np.array(path)

        pts[:, 0] = pts[:, 0] / self.map_resolution + 0.5 * map_w
        pts[:, 1] = pts[:, 1] / self.map_resolution + 0.5 * map_h

        return pts.astype(np.int32)

    def publish_polygon(self, contours):
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

            marker.color.r = float(i % 3 == 0)
            marker.color.g = float(i % 3 == 1)
            marker.color.b = float(i % 3 == 2)
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
        marker_array = MarkerArray()

        for path in self.paths:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()

            marker.ns = "decomposition"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.02

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            for p in path:
                pt = PoseStamped().pose.position
                pt.x = p[0]
                pt.y = p[1]
                pt.z = 0.0
                marker.points.append(pt)

            marker_array.markers.append(marker)

        self.path_publisher.publish(marker_array)

    def debug_save(self, contours):
        img = cv2.normalize(self.map.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX)
        img = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

        path = os.path.join(os.path.dirname(__file__), "map.png")
        cv2.imwrite(path, img)

    def publish_info(self):
        if self.poly_map:
            self.polymap_publisher.publish()

        if self.decomp_map:
            self.decomp_publisher.publish()

        if self.paths:
            self.paths_publisher.publish()


class ReactiveNavigator():
    def __init__(self):
        self.navigator_type = 'reactive'