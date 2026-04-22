#!/usr/bin/env python

import time 
import numpy as np
import copy
from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon
from shapely.ops import unary_union
import cv2
from scipy.spatial import ConvexHull
from scipy.optimize import minimize

########################################################################## Map Manager ###########################################
class Polygon3D:
    def __init__(self, vertices):
        self.vertices = np.array(vertices)
        self.normal = self.compute_normal()
        self.plane_eq = self.compute_plane_equation()
        self.area = self.compute_3d_area()
        self.xy_projection = ShapelyPolygon(self.vertices[:, :2])
        if not self.xy_projection.is_valid:
            raise ValueError(f"Invalid polygon with vertices: {vertices}")

    def compute_normal(self):
        if len(self.vertices) < 3:
            return np.array([np.nan, np.nan, np.nan])
        
        p1, p2, p3 = self.vertices[:3]
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        
        if np.linalg.norm(normal) == 0:
            return np.array([np.nan, np.nan, np.nan])
        
        normal = normal / np.linalg.norm(normal)
        
        # Ensure the z component of the normal is non-negative
        if normal[2] < 0:
            normal = -normal

        return normal

    def compute_plane_equation(self):
        if np.any(np.isnan(self.normal)):
            return (np.nan, np.nan, np.nan, np.nan)
        
        point = self.vertices[0]
        a, b, c = self.normal
        d = -np.dot(self.normal, point)
        return (a, b, c, d)

    def compute_3d_area(self):
        if len(self.vertices) < 3:
            return np.nan

        total_area = 0.0
        origin = self.vertices[0]

        for i in range(1, len(self.vertices) - 1):
            v1 = self.vertices[i] - origin
            v2 = self.vertices[i + 1] - origin
            cross_product = np.cross(v1, v2)
            triangle_area = np.linalg.norm(cross_product) / 2
            total_area += triangle_area

        return total_area
    
    def merge_with(self, other):
        all_vertices = np.vstack((self.vertices, other.vertices))
        hull = ConvexHull(all_vertices[:, :2])
        hull_vertices = all_vertices[hull.vertices]
        xy_hull_vertices = hull_vertices[:, :2]

        new_normal = (self.normal * self.area + other.normal * other.area) / (self.area + other.area)
        
        if np.linalg.norm(new_normal) == 0:
            return self  # Do not merge if normal is invalid
        
        new_normal /= np.linalg.norm(new_normal)

        # Compute d directly by averaging distances to the plane
        all_distances = np.dot(all_vertices, new_normal)
        d_opt = -np.mean(all_distances)

        new_vertices = np.column_stack((xy_hull_vertices, np.zeros(len(xy_hull_vertices))))
        for i, vertex in enumerate(new_vertices):
            xy_vertex = vertex[:2]
            new_vertices[i, 2] = (-d_opt - new_normal[0] * xy_vertex[0] - new_normal[1] * xy_vertex[1]) / new_normal[2]

        return Polygon3D(new_vertices)

    def compute_z_for_xy(self, xy):
        a, b, c, d = self.plane_eq
        x, y = xy
        return (-d - a * x - b * y) / c

    def to_string(self):
        return f"Vertices: {self.vertices.tolist()}\nNormal: {self.normal.tolist()}\nPlane equation: {self.plane_eq}\nArea: {self.area}\n"

class MapManager:
    def __init__(self, z_threshold=0.02):
        self.polygons = []  # Stores existing polygons
        self.z_threshold = z_threshold  # Distance threshold for Z-axis merging
        self.z_drift = 0.0  # Estimated Z-axis drift
        self.single_frame = []

    def add_polygon_list(self, new_polygon_list):
        # 1. Convert incoming polygon vertices to Polygon3D objects
        new_polygons = []
        for vertices in new_polygon_list:
            try:
                new_polygon = Polygon3D(vertices)
                if np.any(np.isnan(new_polygon.normal)):
                    print("Skipping invalid polygon with vertices:", vertices)
                    continue
                new_polygons.append(new_polygon)
            except ValueError as e:
                print(e)

        # 2. Initially compensate new polygons with the last z_drift
        compensated_new_polygons = [self.apply_z_drift(polygon, self.z_drift) for polygon in new_polygons]

        # 3. Find overlapping parts between new and existing polygons to prepare for z_drift optimization
        merge_candidates = []
        for new_polygon in compensated_new_polygons:
            for existing_polygon in self.polygons:
                if self.is_overlap_in_xy(new_polygon, existing_polygon):
                    z_distance = abs(self.compute_average_z_distance(new_polygon, existing_polygon))
                    if z_distance <= self.z_threshold:
                        merge_candidates.append((new_polygon, existing_polygon))
                        break

        # 4. If there are overlapping polygons, optimize z_drift
        if merge_candidates:
            # print("1 self.z_drift (before optimization):", self.z_drift)
            z_drift_increment = self.optimize_z_drift(merge_candidates)  # Obtain increment
            self.z_drift += z_drift_increment  # Update z_drift incrementally
            # print("2 self.z_drift (after optimization):", self.z_drift)

        # 5. Re-compensate new polygons with the updated z_drift
        compensated_new_polygons = [self.apply_z_drift(polygon, self.z_drift) for polygon in new_polygons]
        
        # 6. Merge compensated polygons into the map manager
        for new_polygon in compensated_new_polygons:
            self.merge_polygon(new_polygon)

    def apply_z_drift(self, polygon, z_drift):
        """Apply Z-axis drift compensation to a polygon"""
        compensated_vertices = polygon.vertices.copy()
        compensated_vertices[:, 2] += z_drift  # Compensate only on Z-axis
        return Polygon3D(compensated_vertices)

    def merge_polygon(self, new_polygon):
        """Merge the new polygon with existing polygons"""
        try:
            if np.any(np.isnan(new_polygon.normal)):
                print("Skipping invalid polygon with vertices:", new_polygon.vertices)
                return

            merged = False
            while True:
                to_merge = None
                for i, existing_polygon in enumerate(self.polygons):
                    if self.is_overlap_in_xy(new_polygon, existing_polygon):
                        z_distance = abs(self.compute_average_z_distance(new_polygon, existing_polygon))
                        if z_distance <= self.z_threshold:
                            to_merge = existing_polygon
                            break

                if to_merge is not None:
                    new_polygon = new_polygon.merge_with(to_merge)
                    self.polygons.remove(to_merge)
                    merged = True
                else:
                    break

            self.polygons.append(new_polygon)
            if not merged:
                self.merge_polygons()
        except ValueError as e:
            print(e)

    def is_overlap_in_xy(self, poly1, poly2):
        """Check if two polygons overlap in the XY plane"""
        return poly1.xy_projection.intersects(poly2.xy_projection)

    # def compute_average_z_distance(self, poly1, poly2):
    #     """Calculate the average Z-axis height difference over the overlap region in XY plane"""
    #     intersected_points = poly1.xy_projection.intersection(poly2.xy_projection)
    #     if intersected_points.is_empty:
    #         return float('inf')

    #     points = []
    #     if isinstance(intersected_points, MultiPolygon):
    #         for sub_poly in intersected_points:
    #             points.extend(sub_poly.exterior.coords)
    #     else:
    #         points = list(intersected_points.exterior.coords)

    #     points = np.array(points)

    #     if points.size == 0:
    #         return float('inf')

    #     z_distances = []
    #     for point in points:
    #         xy = point[:2]
    #         z1 = poly1.compute_z_for_xy(xy)
    #         z2 = poly2.compute_z_for_xy(xy)
    #         z_distances.append(abs(z1 - z2))
            
    #     return np.mean(z_distances)
    def compute_average_z_distance(self, poly1, poly2):
        """Calculate the average Z-axis height difference over the overlap region in XY plane"""
        intersected_area = poly1.xy_projection.intersection(poly2.xy_projection)
        
        if intersected_area.is_empty:
            return float('inf')
        
        # Extract all boundary points from the intersection
        boundary_points = []
        
        if isinstance(intersected_area, MultiPolygon):
            # MultiPolygon: collect points from all sub-polygons
            for polygon in intersected_area.geoms:
                boundary_points.extend(polygon.exterior.coords[:-1])  # Exclude repeated closing point
        else:
            # Single Polygon
            boundary_points.extend(intersected_area.exterior.coords[:-1])
        
        if not boundary_points:
            return float('inf')
        
        # Calculate Z differences
        z_differences = []
        for point in boundary_points:
            try:
                z1 = poly1.compute_z_for_xy(point[:2])
                z2 = poly2.compute_z_for_xy(point[:2])
                z_differences.append(abs(z1 - z2))
            except (AttributeError, ValueError):
                continue  # Handle cases where z computation fails
        
        if not z_differences:
            return float('inf')
        
        return np.mean(z_differences)

    def optimize_z_drift(self, merge_candidates):
        """Optimize Z-axis drift to minimize height difference for overlapping polygon pairs"""

        def cost_function(z_drift_increment):
            total_cost = 0.0
            for new_polygon, existing_polygon in merge_candidates:
                # Apply z_drift_increment to the new polygon
                compensated_polygon = self.apply_z_drift(new_polygon, self.z_drift + z_drift_increment)
                # Calculate Z height difference in overlap region
                z_distance = self.compute_average_z_distance(compensated_polygon, existing_polygon)
                # Weight by area of new polygon
                area = new_polygon.area
                # Cost = area-weighted square of height difference
                total_cost += area * z_distance ** 2
            return total_cost

        # Optimize z_drift increment using scipy.optimize.minimize
        result = minimize(cost_function, 0.0, method='BFGS')
        return result.x[0]  # Return optimized z_drift increment

    def merge_polygons(self):
        """Merge all overlapping polygons in the existing set"""
        i = 0
        while i < len(self.polygons):
            polygon = self.polygons[i]
            j = i + 1
            while j < len(self.polygons):
                other_polygon = self.polygons[j]
                if self.is_overlap_in_xy(polygon, other_polygon):
                    z_distance = abs(self.compute_average_z_distance(polygon, other_polygon))
                    if z_distance <= self.z_threshold:
                        merged_polygon = polygon.merge_with(other_polygon)
                        self.polygons[i] = merged_polygon
                        self.polygons.pop(j)
                        # Re-check the merged polygon
                        j = i + 1
                    else:
                        j += 1
                else:
                    j += 1
            i += 1

    def plot_polygons(self):
        img = np.ones((600, 600, 3), dtype=np.uint8) * 255
        all_points = np.vstack([poly.vertices[:, :2] for poly in self.polygons])
        min_x, min_y = np.min(all_points, axis=0)
        max_x, max_y = np.max(all_points, axis=0)
        scale_x = 500 / (max_x - min_x)
        scale_y = 500 / (max_y - min_y)
        scale = min(scale_x, scale_y)
        offset_x = (600 - (max_x - min_x) * scale) / 2
        offset_y = (600 - (max_y - min_y) * scale) / 2

        for poly in self.polygons:
            pts = np.array(poly.xy_projection.exterior.coords, np.float32)
            pts[:, 0] = (pts[:, 0] - min_x) * scale + offset_x
            pts[:, 1] = (max_y - pts[:, 1]) * scale + offset_y  # Flip y coordinate
            pts = pts.astype(np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        return img

    def save_polygons_to_file(self, filename):
        with open(filename, 'w') as f:
            i = 0
            for polygon in self.polygons:
                f.write(str(i) + "\n")
                i += 1
                f.write(polygon.to_string() + "\n")
