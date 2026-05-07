import numpy as np
import cv2

import rclpy

# from navigator_types import SystematicNavigator, ReactiveNavigator
# import utils as ut
# from utils import LogType

from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator
import mirte_lc_nav2.utils as ut
from mirte_lc_nav2.utils import LogType

import trajgenpy as tjp
from trajgenpy import Geometries

import shapely
from shapely.validation import explain_validity


class StraightLinePath(SystematicNavigator):
    def __init__(self, node, resolution=0.1, length=2.0):
        super().__init__(node, resolution)
        self.name = "StraightLinePath"
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
    def __init__(self, node=None, resolution=0.1):
        self.node = node
        self.name = "BousPath"
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

        offset = Geometries.get_sweep_offset(overlap=0.0, height=0.3, field_of_view=90)
        result = []
        self.paths = []

        ut.log(self.node, LogType.INFO, "generating full trajectory")
        for cell in self.cells:
            sweeps = Geometries.generate_sweep_pattern(
                cell, offset, clockwise=False, connect_sweeps=True
            )
            paths_mls = Geometries.GeoMultiTrajectory(sweeps).get_geometry()
            self.paths.append(self.multiline_to_coords(paths_mls))
            result.extend(sweeps)

        # mls = Geometries.GeoMultiTrajectory(result).get_geometry()
        # self.paths = self.multiline_to_coords(mls)

    def multiline_to_coords(self, multiline):
        coords = []
        if isinstance(multiline, shapely.MultiLineString):
            for line in multiline.geoms:
                coords.extend(list(line.coords))
        else:
            for line in multiline:
                coords.extend(list(line.coords))

        return coords

    def generate_path(self, generate, decompose):
        if decompose: self.bcd()
        if generate: self.bous_path()


class SpiralPath(SystematicNavigator):
    def __init__(self, resolution=0.1):
        super().__init__(resolution)
        self.name = "SpiralPath"
        self.center = None
        self.radius = None

    def generate_path(self):
        pass
