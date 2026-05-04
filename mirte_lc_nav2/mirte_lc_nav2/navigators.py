'''
    Navigators
    for Mirte LC Navigation System
    This module defines systematic path planners for the Mirte LC navigation system. 
    It includes implementations of the Bous path and Spiral path planners, 
    which can be used to generate efficient paths for coverage tasks. 
    The module also provides a factory function to retrieve 
    the desired path planner based on a string identifier.    
'''

import numpy as np
import cv2

import rclpy
from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator

import trajgenpy as tjp
from trajgenpy import Geometries

import shapely
from shapely.validation import explain_validity


def get_path_planner(name):
    path_planners = {
        "bous": BousPath,
        "spiral": SpiralPath,
        "straightline": StraightLinePath,
    }
    if name in path_planners.keys():
        return path_planners[name]
    else:
        raise ValueError(f"Unknown path planner: {name}")

class StraightLinePath(SystematicNavigator):
    def __init__(self,node, resolution=0.1, length=2.0):
        super().__init__(node, resolution)
        self.name = "StraightLinePath"
        self.length = length  # meters
        self.start_pose = (0,0,0)

    def generate_path(self):
        if self.start_pose is None:
            return None

        x0, y0, yaw = self.start_pose

        # number of points based on resolution
        n_points = int(self.length / self.resolution)

        path = []
        for i in range(n_points):
            d = i * self.resolution
            x = x0 + d
            y = y0 + d

            # store as (x, y, yaw)
            path.append([x, y, yaw])

        self.path = np.array(path)

class BousPath(SystematicNavigator):
    def __init__(self, node, resolution=0.1):
        self.node = node
        self.name = "BousPath"
        super().__init__(node, resolution)

    def bcd(self):        
        self.node.get_logger().info(f"Contours: {len(self.polymap)}")
        for i, c in enumerate(self.polymap):
            self.node.get_logger().info(f"{i}: {len(c)} points")
        polygons = self.polymap.copy()
        outer_poly = shapely.Polygon(polygons[0])
        
        if not outer_poly.is_valid:
            self.node.get_logger().warn(f"Invalid outer polygon: {explain_validity(outer_poly)}")
            outer_poly = outer_poly.buffer(0)
            if not outer_poly.is_valid:
                self.node.get_logger().warn(f"Could not fix outer polygon")
                return False

            if outer_poly.is_empty:
                self.node.get_logger().error("Outer polygon is empty after fix")
                return False

            if outer_poly.geom_type == "MultiPolygon":
                self.node.get_logger().warn("Outer polygon became MultiPolygon, taking largest piece")
                outer_poly = max(outer_poly.geoms, key=lambda p: p.area)

        outer = Geometries.GeoPolygon(
                outer_poly,
                crs="map"
            )

        # outer.set_crs("EPSG:3857")
        polygons.pop(0)
        
        holes = []

        for contour in polygons:
            poly = shapely.Polygon(contour).simplify(0.1, preserve_topology=True)
            geo_contour = Geometries.GeoPolygon(
                poly,
                crs="map"
                )
            if not poly.is_valid:
                self.node.get_logger().warn(f"Invalid polygon: {explain_validity(poly)}")
                poly = poly.buffer(0)
                if not poly.is_valid:
                    self.node.get_logger().warn(f"Could not fix polygon: {poly}")
                    continue
                if poly.is_empty:
                    self.node.get_logger().error("Polygon is empty after fix")
                    return False

                if poly.geom_type == "MultiPolygon":
                    self.node.get_logger().warn("Outer polygon became MultiPolygon, taking largest piece")
                    poly = max(poly.geoms, key=lambda p: p.area)
                
            # geo_contour.set_crs("EPSG:3857")
            holes.append(geo_contour.get_geometry())
        
        polygon_list = Geometries.decompose_polygon(
            outer.get_geometry(), 
            # obstacles=shapely.MultiPolygon(holes)
        )

        self.cells = polygon_list
        self.publish_decomposition()
        self.node.get_logger().info('map decomposed, publishing decomposition')
        
    def bous_path(self, robot_width = 0.3):
        self.node.get_logger().info(f"Number of cells: {len(self.cells)}")
        offset = Geometries.get_sweep_offset(overlap=0.0, height=0.3, field_of_view=90)
        result = []
        self.node.get_logger().info('generating path')
        for decomposed_poly in self.cells:
            sweeps_connected = Geometries.generate_sweep_pattern(
                decomposed_poly, offset, clockwise=True, connect_sweeps=True
            )
            result.extend(sweeps_connected)
        
        mls = Geometries.GeoMultiTrajectory(result, crs="map").get_geometry()
        self.path = self.to_ros_path(self.multiline_to_coords(mls))
        
    def multiline_to_coords(self, multiline):
        coords = []
        for line in multiline.geoms:
            coords.extend(list(line.coords))
        return coords

    def generate_path(self):
        self.bcd()
        self.bous_path()

class SpiralPath(SystematicNavigator):
    def __init__(self, resolution=0.1):
        super().__init__(resolution)
        self.name = "SpiralPath"
        self.center = None
        self.radius = None
    
    def generate_path(self):
        pass
