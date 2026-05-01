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
import fields2cover as f2c

import rclpy

from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator

from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import (Coordinates,
                                       Coordinate,
                                       HeadlandMode, 
                                       SwathMode, 
                                       RowSwathMode, 
                                       RouteMode, 
                                       PathMode)

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
        self.node.nav2coverage_client = rclpy.action.ActionClient(self.node, ComputeCoveragePath, 'compute_coverage_path')
        super().__init__(node, resolution)

    def bcd(self):
        Decomposition is a feature of f2c 2.0 which 
        is incompatible with Opennav2 coverage for ros2 humble
        
        decomposer = f2c.DECOMP_Boustrophedon()
        self.cells = decomposer.decompose(self.field)
        self.publish_decomposition()
        
    def bous_path(self, robot_width = 0.3):
        goal = ComputeCoveragePath.Goal()

        # Add polygons of free space map to message
        polygons = []

        for contour in self.polymap:
            poly = []
            polygon = Coordinates()

            for pt in contour:
                x = pt[0]
                y = pt[1]
                coord = Coordinate()
                coord.axis1 = float(x)
                coord.axis2 = float(y)
                poly.append(coord)

            polygon.coordinates = poly
            polygons.append(polygon)

        goal.polygons = polygons

        # Select Headland mode
        headland_mode = HeadlandMode()
        headland_mode.width = 0.5
        goal.headland_mode = headland_mode

        # Select Swath mode
        swath_mode = SwathMode()
        swath_mode.objective = 'LENGTH'
        swath_mode.mode = 'BRUTE_FORCE'
        goal.swath_mode = swath_mode

        # Select Route mode
        route_mode = RouteMode()
        route_mode.mode = 'BOUSTROPHEDON'
        goal.route_mode = route_mode

        # Select Path mode
        path_mode = PathMode()
        path_mode.mode = 'DUBIN'
        path_mode.continuity_mode = 'DISCONTINUOUS'
        path_mode.turn_point_distance = 0.1
        goal.path_mode = path_mode   
        
        # send goal and receive response
        send_goal_future = self.node.nav2coverage_client.send_goal_async(
            goal,
            feedback_callback=self._feedbackCallback
        )

        send_goal_future.add_done_callback(self._goal_response_callback)
        return True
    
    def _goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("Coverage goal rejected")
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        result = future.result().result
        self.nav_path = result.nav_path
        self.node.get_logger().info("Coverage path received")
        self.node.path_publisher()
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback
    
    def generate_path(self):
        # self.bcd()
        self.bous_path()

class SpiralPath(SystematicNavigator):
    def __init__(self, resolution=0.1):
        super().__init__(resolution)
        self.name = "SpiralPath"
        self.center = None
        self.radius = None
    
    def generate_path(self):
        pass
