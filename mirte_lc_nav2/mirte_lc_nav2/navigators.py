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

from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator

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
        self.name = "BousPath"
        super().__init__(node, resolution)

    def bcd(self):
        decomposer = f2c.DECOMP_Boustrophedon()
        self.cells = decomposer.decompose(self.field)
        self.publish_decomposition()
        
    def bous_path(self, robot_width = 0.3):
        if self.field is None:
            return None

        robot_width = 0.3  # adjust

        # 1. Swath generation
        swath_gen = f2c.SG_BruteForce()
        swaths = swath_gen.generateSwaths(self.cells, robot_width)

        # 2. Path planning
        path_planner = f2c.PP_PathPlanning()
        dubins_cc = f2c.PP_DubinsCurvesCC()
        path = path_planner.planPath(swaths, dubins_cc)

        # 3. Convert to numpy path
        coords = []
        for state in path.states:
            x = state.point.getX()
            y = state.point.getY()
            yaw = state.angle
            coords.append([x, y, yaw])

        self.path = np.array(coords)
    
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
