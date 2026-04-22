#!/usr/bin/env python

import numpy as np
import copy
import cv2
#import pyrealsense2 as rs
from skimage import morphology
import cupy as cp
import scipy.ndimage

def transform_point(point, T):
    """ Transform a point from camera coordinates to world coordinates. """
    point_h = np.append(point, 1)  # Convert point to homogeneous coordinates
    transformed_point_h = np.dot(T, point_h)
    return transformed_point_h[:3]

def transform_vector(vector, T):
    """ Transform a vector from camera coordinates to world coordinates. """
    # Only consider the rotation part, so we take the upper-left 3x3 part of T
    R = T[:3, :3]
    transformed_vector = np.dot(R, vector)
    if transformed_vector[2] < 0:
        transformed_vector = -transformed_vector
    return transformed_vector

def angle_between_vectors(v1, v2):
    """ Calculate the angle (in radians) between two vectors. """
    dot_product = np.dot(v1, v2)
    norms = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos_angle = dot_product / norms
    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
    return angle

def process_polygon(vertices, normal, T):
    """
    Args:
    - vertices: Polygon vertices in camera coordinates, Nx3 numpy array
    - normal: Normalized normal vector of the polygon plane, 3-element numpy array
    
    Returns:
    - angle: Angle between the normal vector and the z-axis in world coordinates (degrees)
    - world_vertices: Polygon vertices in world coordinates
    """
    
    # Transform the normal vector from camera coordinates to world coordinates
    world_normal = transform_vector(normal, T)

    # Calculate the angle between the normal vector and the z-axis in world coordinates
    world_z_axis = np.array([0, 0, 1])
    angle_radians = angle_between_vectors(world_normal, world_z_axis)
    angle_degrees = np.degrees(angle_radians)

    # Check if the angle is less than 25°
    thresh_angle = 25
    if angle_degrees < thresh_angle:
        # Transform vertices from camera coordinates to world coordinates
        world_vertices = np.array([transform_point(vertex, T) for vertex in vertices])
        return angle_degrees, world_vertices
    else:
        return angle_degrees, None

def project_points_to_plane(plane_eq, fx, fy, cx, cy, uv_coords):
    """
    Projects points from image coordinates (u, v) to a specified plane in camera coordinates.

    Args:
    - plane_eq (tuple): Coefficients (A, B, C, D) of the plane equation Ax + By + Cz + D = 0.
    - fx, fy, cx, cy: Camera intrinsic parameters.
    - uv_coords (np.ndarray): Array of image coordinates (u, v) with shape (N, 2).

    Returns:
    - np.ndarray: Array of 3D coordinates (x, y, z) projected onto the plane with shape (N, 3).
    """
    A, B, C, D = plane_eq
    points_3d = []

    for (u, v) in uv_coords:
        # Convert image coordinates to normalized camera coordinates
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy

        # Calculate the z-coordinate using the plane equation
        z = -D / (A * x_norm + B * y_norm + C)
        x = x_norm * z
        y = y_norm * z

        points_3d.append([x, y, z])

    return np.array(points_3d)

# def depth_to_point_cloud_region(depth_map, polygon_vertices, fx, fy, cx, cy):
#     """Generate a point cloud for a region specified by polygon vertices in the depth map."""
#     height, width = depth_map.shape
#     mask = np.zeros(depth_map.shape, dtype=np.uint8)
#     cv2.fillPoly(mask, [polygon_vertices], 1)
    
#     mask = cp.asarray(mask)
#     depth_map = cp.asarray(depth_map)
    
#     indices = cp.where(mask == 1)
#     z = 0.00025 * depth_map[indices]
#     i, j = indices[1], indices[0]
#     x = (i - cx) * z / fx
#     y = (j - cy) * z / fy
#     point_cloud = cp.stack((x, y, z), axis=-1)
#     return point_cloud

def depth_to_point_cloud_region(depth_map, polygon_vertices, fx, fy, cx, cy):
    """Generate a point cloud for a region specified by polygon vertices in the depth map."""
    height, width = depth_map.shape
    mask = np.zeros((height, width), dtype=np.uint8)

    # ✅ 检查 polygon_vertices 是否为空或非法
    if polygon_vertices is None or len(polygon_vertices) < 3:
        print("[Warn] Invalid polygon passed to fillPoly.")
        return cp.empty((0, 3), dtype=cp.float32)

    # ✅ 转换为 OpenCV fillPoly 接受的格式：int32 + (N,1,2)
    try:
        polygon_int = polygon_vertices.astype(np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(mask, [polygon_int], 1)
    except Exception as e:
        print(f"[Error] fillPoly failed: {e}")
        return cp.empty((0, 3), dtype=cp.float32)

    # ✅ 转换为 CuPy 格式
    mask_cp = cp.asarray(mask)
    depth_cp = cp.asarray(depth_map)

    # ✅ 提取掩码区域坐标
    indices = cp.where(mask_cp == 1)
    z = 0.00025 * depth_cp[indices]  # 单位换算（毫米 → 米）
    i, j = indices[1], indices[0]
    x = (i - cx) * z / fx
    y = (j - cy) * z / fy
    point_cloud = cp.stack((x, y, z), axis=-1)
    return point_cloud

def fit_plane_ransac(points, max_trials=100, min_samples=3, residual_threshold=0.005, outlier_threshold=0.15):
    """Estimate a plane from points using RANSAC with Cupy for GPU acceleration."""
    best_model = None
    best_inliers = 0

    # Pre-generate random samples
    sample_indices = cp.random.choice(points.shape[0], (max_trials, min_samples), replace=False)
    sample_points = points[sample_indices]
    
    normals = cp.zeros((max_trials, 3))
    ds = cp.zeros(max_trials)
    num_inliers = cp.zeros(max_trials)
    
    for i in range(max_trials):
        if sample_points[i].shape[0] < 3:
            continue
        
        p1, p2, p3 = sample_points[i][:3]
        v1 = p2 - p1
        v2 = p3 - p1
        normal = cp.cross(v1, v2)
        normal = normal / cp.linalg.norm(normal)
        
        d = -cp.dot(normal, p1)
        
        normals[i] = normal
        ds[i] = d
    
    distances = cp.abs(cp.dot(points, normals.T) + ds)
    inliers = distances < residual_threshold
    num_inliers = cp.sum(inliers, axis=0)
    
    best_index = cp.argmax(num_inliers)
    best_inliers = num_inliers[best_index]
    
    if best_inliers == 0:
        raise ValueError("Could not find a valid plane")

    best_model = (normals[best_index], ds[best_index])
    
    normal, d = best_model
    outlier_ratio = 1 - (best_inliers / points.shape[0])
    
    if outlier_ratio > outlier_threshold:
        raise ValueError(f"Outlier ratio too high: {outlier_ratio:.2f}, threshold: {outlier_threshold}")

    if d > 0:
        normal = -normal
        d = -d
        
    return cp.asnumpy(normal), cp.asnumpy(d)

def anisotropic_diffusion(depth_img, num_iter=20, kappa=50, gamma=0.1, option=1):
    """Perform anisotropic diffusion on a depth image using GPU with Cupy."""
    num_iter = cv2.getTrackbarPos('num_iter', 'smoothed_depth_img')
    kappa = cv2.getTrackbarPos('kappa', 'smoothed_depth_img')
    gamma = 0.1 * cv2.getTrackbarPos('gamma', 'smoothed_depth_img')
    img = cp.array(depth_img, dtype=cp.float32)
    h, w = img.shape

    for i in range(num_iter):
        # Compute gradients
        deltaN = cp.zeros_like(img)
        deltaS = cp.zeros_like(img)
        deltaE = cp.zeros_like(img)
        deltaW = cp.zeros_like(img)
        
        deltaN[1:, :] = img[:-1, :] - img[1:, :]
        deltaS[:-1, :] = img[1:, :] - img[:-1, :]
        deltaE[:, :-1] = img[:, 1:] - img[:, :-1]
        deltaW[:, 1:] = img[:, :-1] - img[:, 1:]

        # Compute diffusion coefficients
        if option == 1:
            cN = cp.exp(-(deltaN / kappa) ** 2)
            cS = cp.exp(-(deltaS / kappa) ** 2)
            cE = cp.exp(-(deltaE / kappa) ** 2)
            cW = cp.exp(-(deltaW / kappa) ** 2)
        elif option == 2:
            cN = 1. / (1. + (deltaN / kappa) ** 2)
            cS = 1. / (1. + (deltaS / kappa) ** 2)
            cE = 1. / (1. + (deltaE / kappa) ** 2)
            cW = 1. / (1. + (deltaW / kappa) ** 2)

        # Update image
        img += gamma * (cN * deltaN + cS * deltaS + cE * deltaE + cW * deltaW)

    return cp.asnumpy(img)

def depth_to_normals(depth_img, fx, fy, cx, cy):
    """Calculate surface normals from a depth image using the Sobel operator."""
    du = cv2.Sobel(depth_img, cv2.CV_64F, 1, 0, ksize=5)
    dv = cv2.Sobel(depth_img, cv2.CV_64F, 0, 1, ksize=5)
    
    X_u = np.ones_like(depth_img, dtype=np.float64) / fx
    Y_v = np.ones_like(depth_img, dtype=np.float64) / fy
    Z_u = du
    Z_v = dv

    # Calculate normal vector components
    nx = Y_v * Z_u
    ny = Z_v * X_u
    nz = -(X_u * Y_v)

    # Normalize the normal vectors
    norm = np.sqrt(nx**2 + ny**2 + nz**2 + 1e-8)
    nx /= norm
    ny /= norm
    nz /= norm

    # Handle invalid depth (e.g., zero depth)
    mask = depth_img == 0
    nx[mask] = 0
    ny[mask] = 0
    nz[mask] = 0

    # Map normal vector components to RGB color space in the range [0, 255]
    normals_rgb = cv2.merge(((nx + 1) / 2 * 255, (ny + 1) / 2 * 255, (nz + 1) / 2 * 255)).astype(np.uint8)
    return normals_rgb

def add_border_conditionally(image):
    """Add a border conditionally around the image if certain criteria are met."""
    rows, cols, _ = image.shape
    new_image = image.copy()

    # Create masks to check for border conditions
    mask_top = np.any(image[0, :, :] != [127, 127, 127], axis=-1)
    mask_bottom = np.any(image[-1, :, :] != [127, 127, 127], axis=-1)
    mask_left = np.any(image[:, 0, :] != [127, 127, 127], axis=-1)
    mask_right = np.any(image[:, -1, :] != [127, 127, 127], axis=-1)

    # Apply conditional border
    new_image[0, mask_top] = [255, 255, 255]
    new_image[-1, mask_bottom] = [255, 255, 255]
    new_image[mask_left, 0] = [255, 255, 255]
    new_image[mask_right, -1] = [255, 255, 255]

    return new_image
