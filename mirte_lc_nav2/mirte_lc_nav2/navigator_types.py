'''
    Navigator types
'''

import numpy as np
import cv2
import fields2cover as f2c
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os

class SystematicNavigator():
    def __init__(self, node, resolution=0.1, map_resolution=0.05, lethal_threshold=20.0):
        '''
        Systematic navigator class
        '''
        self.navigator_type = 'systematic'
        self.path = None
        self.map = None
        self.polymap = None
        self.threshold = lethal_threshold
        self.resolution = resolution
        self.map_resolution = map_resolution
        self.node = node
        
        self.polymap_publisher = self.node.create_publisher(
            PolygonStamped, f'/systematic_navigator/map_contours', 10)

        self.decomp_publisher = self.node.create_publisher(
            MarkerArray, f'/systematic_navigator/decomposed_map', 10)
   
    def update_map(self, new_map, show=False):
        """update internal map of the navigator"""

        self.map = new_map

        # Convert costmap to binary grid
        self.binary_costmap = np.zeros_like(self.map, dtype=np.uint8)

        free_mask = np.zeros_like(self.map, dtype=np.uint8)
        free_mask[self.map < self.threshold] = 255
        free_mask[self.map == -1] = 0
                
        margin_m = 0.3  # desired margin in meters
        margin_px = max(1, int(margin_m / self.map_resolution))

        # make kernel odd-sized (cleaner result)
        if margin_px % 2 == 0:
            margin_px += 1

        kernel = np.ones((margin_px, margin_px), np.uint8)
        free_mask = cv2.erode(free_mask, kernel)

        contours, _ = cv2.findContours(free_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if show:
            self.debug_save(free_mask, contours)

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        cell = f2c.Cell()
        closed_contour = []
        for contour in contours:
            ring = f2c.LinearRing()
            pts = contour[:, 0, :]
            # convert points
            map_height = self.map.shape[0]
            map_width = self.map.shape[1]
            
            points = [((px - 0.5*map_width) * self.map_resolution, (py - 0.5*map_height) * self.map_resolution) for px, py in pts]

            # ensure closure
            if points[0] != points[-1]:
                points.append(points[0])

            for x, y in points:
                ring.addPoint(f2c.Point(x, y))
            
            closed_contour.append(points)
            cell.addGeometry(ring)
        self.publish_polygon(closed_contour)

        cells = f2c.Cells(cell)
        self.polymap = closed_contour
        self.field = f2c.Field(cells)

        self.generate_path()
        return None

    def publish_polygon(self, contours):
        msg = PolygonStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for contour in contours:
            pts = contour
            for px, py in pts:
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

            marker.color.r = (i % 3 == 0)
            marker.color.g = (i % 3 == 1)
            marker.color.b = (i % 3 == 2)
            marker.color.a = 1.0

            ring = cell.getGeometry().getOuterRing()

            for j in range(ring.size()):
                p = ring.getPoint(j)

                pt = PoseStamped().pose.position
                pt.x = p.getX()
                pt.y = p.getY()
                pt.z = 0.0

                marker.points.append(pt)

            # close the loop
            if ring.size() > 0:
                first = ring.getPoint(0)
                pt = PoseStamped().pose.position
                pt.x = first.getX()
                pt.y = first.getY()
                pt.z = 0.0
                marker.points.append(pt)

            marker_array.markers.append(marker)

        self.decomp_publisher.publish(marker_array)
        
    def debug_save(self, free_mask, contours):
        img = cv2.normalize(self.map.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX)
        img = img.astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

        # save next to script
        path = os.path.join(os.path.dirname(), "map.png")
        cv2.imwrite(path, img)
        
    def publish_info(self, poly_map=True, decomp_map=True, path=True):
        if poly_map:
            self.polymap_publisher.publish()
        
        if decomp_map:
            self.decomp_publisher.publish()
        
        if path:
            self.path_publisher.publish()

    def generate_path(self):
        pass

class ReactiveNavigator():
    def __init__(self):
        self.navigator_type = 'reactive'