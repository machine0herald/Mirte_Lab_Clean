import numpy as np
import cv2

import rclpy
from mirte_lc_nav2.navigator_types import SystematicNavigator, ReactiveNavigator
import utils as ut

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
    if name in path_planners:
        return path_planners[name]
    raise ValueError(f"Unknown path planner: {name}")


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

        self.path = np.array(path)


class BousPath(SystematicNavigator):
    def __init__(self, node, resolution=0.1):
        self.node = node
        self.name = "BousPath"
        super().__init__(node, resolution)

    def bcd(self):
        if self.node is not None:
            ut.log(self.node, ut.INFO, f"Contours: {len(self.polymap)}")

        for i, c in enumerate(self.polymap):
            ut.log(self.node, ut.INFO, f"{i}: {len(c)} points")

        polygons = self.polymap.copy()
        outer_poly = shapely.Polygon(polygons[0])

        if not outer_poly.is_valid:
            ut.log(self.node, ut.WARN, f"Invalid outer polygon: {explain_validity(outer_poly)}")
            outer_poly = outer_poly.buffer(0)

            if not outer_poly.is_valid:
                ut.log(self.node, ut.WARN, "Could not fix outer polygon")
                return False

            if outer_poly.is_empty:
                ut.log(self.node, ut.ERR, "Outer polygon is empty after fix")
                return False

            if outer_poly.geom_type == "MultiPolygon":
                ut.log(self.node, ut.WARN, "Outer polygon became MultiPolygon, taking largest piece")
                outer_poly = max(outer_poly.geoms, key=lambda p: p.area)

        outer = Geometries.GeoPolygon(outer_poly, crs="map")
        polygons.pop(0)

        holes = []

        for contour in polygons:
            poly = shapely.Polygon(contour).simplify(0.1, preserve_topology=True)

            if not poly.is_valid:
                ut.log(self.node, ut.WARN, f"Invalid polygon: {explain_validity(poly)}")
                poly = poly.buffer(0)

                if not poly.is_valid:
                    ut.log(self.node, ut.WARN, "Could not fix polygon")
                    continue

                if poly.is_empty:
                    ut.log(self.node, ut.ERR, "Polygon is empty after fix")
                    return False

                if poly.geom_type == "MultiPolygon":
                    ut.log(self.node, ut.WARN, "Polygon became MultiPolygon, taking largest piece")
                    poly = max(poly.geoms, key=lambda p: p.area)

            holes.append(poly)

        polygon_list = Geometries.decompose_polygon(outer.get_geometry())
        self.cells = polygon_list

        if self.node is not None:
            self.publish_decomposition()
            ut.log(self.node, ut.INFO, "map decomposed, publishing decomposition")

    def bous_path(self, robot_width=0.3):
        ut.log(self.node, ut.INFO, f"Number of cells: {len(self.cells)}")
        ut.log(self.node, ut.INFO, "generating path")

        offset = Geometries.get_sweep_offset(overlap=0.0, height=0.3, field_of_view=90)

        result = []
        for cell in self.cells:
            sweeps = Geometries.generate_sweep_pattern(
                cell, offset, clockwise=True, connect_sweeps=True
            )
            result.extend(sweeps)

        mls = Geometries.GeoMultiTrajectory(result, crs="map").get_geometry()
        self.path = self.multiline_to_coords(mls)

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