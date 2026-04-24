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
from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator

def get_path_planner(name):
    path_planners = {
        "bous": BousPath,
        "spiral": SpiralPath,
        "straightline": StraightLinePath,
    }
    if name in path_planners.keys():
        return path_planners[name]()
    else:
        raise ValueError(f"Unknown path planner: {name}")

class StraightLinePath(SystematicNavigator):
    def __init__(self, resolution=0.1, length=2.0):
        super().__init__(resolution)
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
            x = x0 + d * np.cos(yaw)
            y = y0 + d * np.sin(yaw)

            # store as (x, y, yaw)
            path.append([x, y, yaw])

        self.path = np.array(path)

class BousPath(SystematicNavigator):
    def __init__(self, resolution=0.1):
        self.name = "BousPath"
        super().__init__(resolution)

    def partition_map(self):
        if self.map is None:
            return None
        # Example partitioning logic (this can be replaced with actual partitioning)
        self.partitions = np.array_split(self.map, 4)  # Split the map into 4 partitions

    def bous_path(self):
        # Generate back and forth path within each partition
        for partition in self.partitions:
            pass
        
        # Connect the paths between partitions
        self.path = np.concatenate(self.partitions)
    
    def generate_path(self):
        self.partition_map()
        self.bous_path()

class SpiralPath(SystematicNavigator):
    def __init__(self, resolution=0.1):
        super().__init__(resolution)
        self.name = "SpiralPath"
        self.center = None
        self.radius = None
    
    def update_spiral_parameters(self, center, radius):
        self.center = center
        self.radius = radius
    
    def generate_path(self):
        if self.center is None or self.radius is None:
            return None
        angles = np.linspace(0, 4 * np.pi, num_points)
        self.path = np.array([
            [self.center[0] + self.radius * np.cos(angle), 
             self.center[1] + self.radius * np.sin(angle)] 
            for angle in angles
        ])