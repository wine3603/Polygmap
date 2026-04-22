#!/usr/bin/env python3

import time
import rospy
import rospkg
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
import tf.transformations
from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon
from shapely.ops import unary_union
import cv2
from scipy.spatial import ConvexHull
import copy
from polygon_mapping.msg import Polygon3D as Polygon3D_msg
from polygon_mapping.msg import MapManager as MapManager_msg
from geometry_msgs.msg import Point32, Vector3
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import tf.transformations as tf_trans

from modules.mapmanager import Polygon3D, MapManager
from modules.image_processes_cupy import (
    process_polygon, project_points_to_plane, depth_to_point_cloud_region,
    fit_plane_ransac, anisotropic_diffusion, depth_to_normals, add_border_conditionally
)

#import pyrealsense2 as rs
from skimage import morphology
import os
import re
import glob

# Global transformation matrix
T = np.eye(4)
T_lock = np.eye(4)

class PosePublishNode:
    def __init__(self, map_manager, frequency=30.0):
        # Initialize the ROS node
        rospy.init_node('polygon_node', anonymous=True)
        global T

        # Create TF buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Link MapManager to ROS
        self.map_manager = map_manager
        self.frequency = frequency
        self.pub = rospy.Publisher('/map_manager', MapManager_msg, queue_size=5)
        self.marker_pub = rospy.Publisher('/polygon_markers', Marker, queue_size=100)

        # Initialize pose and path publishers
        self.pose_pub = rospy.Publisher('/camera_pose', PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/camera_path', Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "world"

        self.pose_raw_pub = rospy.Publisher('/camera_pose_raw', PoseStamped, queue_size=1)
        self.path_raw_pub = rospy.Publisher('/camera_raw_path', Path, queue_size=10)
        self.path_raw = Path()
        self.path_raw.header.frame_id = "world"

        # Drift counters for tracking positive and negative drift during stair climbing
        self.positive_drift_cont = 0
        self.nagetive_drift_cont = 0

        # Timer for polygon publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.publish_polygons)

    def publish_polygons(self, event):
        """Publish global map with polygons."""
        msg = MapManager_msg()
        for polygon in self.map_manager.polygons:
            poly_msg = Polygon3D_msg()
            for vertex in polygon.vertices:
                point = Point32(x=vertex[0], y=vertex[1], z=vertex[2])
                poly_msg.vertices.append(point)

            normal = Vector3(x=polygon.normal[0], y=polygon.normal[1], z=polygon.normal[2])
            poly_msg.normal = normal
            poly_msg.plane_eq = polygon.plane_eq
            poly_msg.area = polygon.area
            msg.polygons.append(poly_msg)

        self.pub.publish(msg)
        global T
        T_lock = copy.deepcopy(T)
        self.publish_camera_pose(T_lock)
        self.publish_markers()

    def publish_markers(self):
        """Publish markers for polygons."""
        for i, polygon in enumerate(self.map_manager.polygons):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "polygons"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.5)  
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0


            for vertex in polygon.vertices:
                point = Point()
                point.x = vertex[0]
                point.y = vertex[1]
                point.z = vertex[2]
                marker.points.append(point)
            if len(polygon.vertices) > 2:
                point = Point()
                point.x = polygon.vertices[0][0]
                point.y = polygon.vertices[0][1]
                point.z = polygon.vertices[0][2]
                marker.points.append(point)  

            self.marker_pub.publish(marker)

    def publish_camera_pose(self, T):
        # Extract translation vector
        translation = T[0:3, 3]
        
        # Extract rotation matrix and convert it to a quaternion
        rotation_matrix = T[0:3, 0:3]
        euler_angles = tf_trans.euler_from_matrix(rotation_matrix, 'sxyz')
        yaw = euler_angles[2] + 3.14159 / 2  # Retain the yaw angle
        
        # Set roll and pitch to 0, reconstruct quaternion
        quaternion = tf_trans.quaternion_from_euler(0, 0, yaw)

        # Apply drift compensation on the Z-axis
        translation[2] += self.map_manager.z_drift

        if self.map_manager.z_drift > 0:
            self.positive_drift_cont += 1
        else:
            self.nagetive_drift_cont += 1

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"  # World coordinate frame

        # Set position
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]

        # Set orientation (quaternion)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Publish camera pose
        self.pose_pub.publish(pose_msg)

        # Update path
        self.path.poses.append(pose_msg)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

        pose_raw_msg = copy.deepcopy(pose_msg)

        # Remove drift compensation for raw pose
        pose_raw_msg.pose.position.z -= self.map_manager.z_drift
        self.pose_raw_pub.publish(pose_raw_msg)

        # Update raw path
        self.path_raw.poses.append(pose_raw_msg)
        self.path_raw.header.stamp = rospy.Time.now()
        self.path_raw_pub.publish(self.path_raw)


def nothing(x):
    """Placeholder function for trackbar."""
    pass

def read_transformation_matrix(t_file_path):
    """Read transformation matrices and timestamps from a file."""
    T_matrices, timestamps = [], []
    current_data = ""
    with open(t_file_path, 'r') as t_file:
        for line in t_file:
            current_data += line.strip() + " "
            numbers = re.findall(r'-?\d+\.\d+e[+-]?\d+|-?\d+\.\d+|-?\d+', current_data)
            if len(numbers) == 18:
                idx = int(numbers[0])
                # idx = int(round(float(numbers[0])))  # 先转浮点，再四舍五入
                T = np.array(numbers[1:17]).astype(float).reshape(4, 4)
                timestamp = float(numbers[-1])
                T_matrices.append(T)
                timestamps.append(timestamp)
                current_data = ""
    return T_matrices, timestamps

def set_cv_config(thresh1=50, thresh2=100, num_iter=60, kappa=134, gamma=2):
    """Set up OpenCV windows and trackbars for image processing parameters."""
    cv2.namedWindow('Filtered Image with Largest Contour')
    cv2.createTrackbar('Canny thresh1', 'Filtered Image with Largest Contour', thresh1, max(thresh1, 1000), nothing)
    cv2.createTrackbar('Canny thresh2', 'Filtered Image with Largest Contour', thresh2, max(thresh2, 1000), nothing)

    cv2.namedWindow('smoothed_depth_img')
    cv2.createTrackbar('num_iter', 'smoothed_depth_img', num_iter, max(num_iter, 360), nothing)
    cv2.createTrackbar('kappa', 'smoothed_depth_img', kappa, max(kappa, 500), nothing)
    cv2.createTrackbar('gamma', 'smoothed_depth_img', gamma, max(gamma, 80), nothing)

def main():
    try:
        # Create map manager
        map_manager = MapManager(z_threshold=0.05)
        node = PosePublishNode(map_manager)
        set_cv_config()

        global T
        fx, fy, cx, cy = 228.686, 230.078, 157.748, 123.968  # Example camera intrinsics

        # Initialize ROS package and get dataset paths
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('polygon_mapping')
        dataset_path = os.path.join(package_path, 'data/', 'dataset_5/')
        rgb_images = sorted(glob.glob(dataset_path + 'rgb/*.png'))
        depth_images = sorted(glob.glob(dataset_path + 'depth/*.png'))

        # Load transformation matrices and timestamps
        T_file_path = os.path.join(dataset_path, 'transformation_matrix.txt')
        T_matrices, timestamps = read_transformation_matrix(T_file_path)

        os.makedirs(os.path.join(dataset_path, "processed","canny"), exist_ok=True)
        os.makedirs(os.path.join(dataset_path, "processed","depth"), exist_ok=True)
        os.makedirs(os.path.join(dataset_path, "processed","normal_raw"), exist_ok=True)
        os.makedirs(os.path.join(dataset_path, "processed","normal_smoothed"), exist_ok=True)
        os.makedirs(os.path.join(dataset_path, "processed","polygon_extracted"), exist_ok=True)
        os.makedirs(os.path.join(dataset_path, "processed","polygon_map"), exist_ok=True)

        # Process each image
        for idx, (T_matrix, timestamp) in enumerate(zip(T_matrices, timestamps)):
            rgb_image_path = os.path.join(dataset_path, 'rgb', f'{str(idx + 1).zfill(6)}.png')
            depth_image_path = os.path.join(dataset_path, 'depth', f'{str(idx + 1).zfill(6)}.png')

            color_image = cv2.imread(rgb_image_path)
            depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
            T = T_matrix
            T_lock = copy.deepcopy(T)

            time_start_loop = time.time()

            # Smooth depth image using anisotropic diffusion
            time_start = time.time()
            smoothed_depth_img = anisotropic_diffusion(depth_image)
            print('Anisotropic diffusion time cost', 1000 * (time.time() - time_start), 'ms')

            # Save processed depth and normal images
            cv2.imwrite(dataset_path + "processed/depth/" + f'{str(idx + 1).zfill(6)}.png',
                        cv2.applyColorMap(cv2.convertScaleAbs(smoothed_depth_img, alpha=0.03), cv2.COLORMAP_JET))
            normals = depth_to_normals(smoothed_depth_img, fx, fy, cx, cy)
            cv2.imwrite(dataset_path + "processed/normal_smoothed/" + f'{str(idx + 1).zfill(6)}.png', normals)
            normals_raw = depth_to_normals(depth_image, fx, fy, cx, cy)
            cv2.imwrite(dataset_path + "processed/normal_raw/" + f'{str(idx + 1).zfill(6)}.png', normals_raw)

            # Retrieve Canny threshold values from trackbars
            thresh1 = cv2.getTrackbarPos('Canny thresh1', 'Filtered Image with Largest Contour')
            thresh2 = cv2.getTrackbarPos('Canny thresh2', 'Filtered Image with Largest Contour')

            # Apply sharpening filter for edge enhancement
            kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
            filtered_img = cv2.filter2D(normals, -1, kernel)
            filtered_img = add_border_conditionally(filtered_img)
            edges = cv2.Canny(filtered_img, thresh1, thresh2)

            # Apply morphological closing to fill edge gaps
            kernel = np.ones((13, 13), np.uint8)
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

            # Find and process contours
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            min_area = 5000
            new_polygon_list = []

            for i, contour in enumerate(contours):
                if hierarchy[0][i][3] == -1:
                    continue
                if cv2.contourArea(contour) < min_area:
                    continue

                hull = contour
                epsilon = 0.02 * cv2.arcLength(hull, True)
                approx = cv2.approxPolyDP(hull, epsilon, True)
                if cv2.contourArea(approx) < min_area:
                    continue

                cv2.drawContours(filtered_img, [approx], 0, (0, 255, 0), 2)
                for point in approx:
                    cv2.circle(filtered_img, tuple(point[0]), 5, (0, 0, 255), -1)

                polygon_points = depth_to_point_cloud_region(smoothed_depth_img, approx.reshape(-1, 2), fx, fy, cx, cy)
                if polygon_points.size == 0:
                    print("No valid points in the polygon region")

                try:
                    normal, d = fit_plane_ransac(
                        polygon_points, max_trials=20, min_samples=3, residual_threshold=0.01
                    )
                    projected_points = project_points_to_plane((*normal, d), fx, fy, cx, cy, approx.reshape(-1, 2))
                    angle, world_vertices = process_polygon(projected_points, normal, T_lock)
                    if world_vertices is not None:
                        new_polygon_list.append(world_vertices)
                except ValueError as e:
                    print("Error in fitting plane:", e)

            map_manager.add_polygon_list(new_polygon_list)

            if map_manager.polygons:
                map_img = map_manager.plot_polygons()
                cv2.imshow('Polygon Map', map_img)
                cv2.imwrite(dataset_path + "processed/polygon_map/" + f'{str(idx + 1).zfill(6)}.png', map_img)

            # cv2.imshow('smoothed_depth_img', cv2.applyColorMap(cv2.convertScaleAbs(smoothed_depth_img, alpha=0.03), cv2.COLORMAP_JET))
            # cv2.imshow('normals Image', normals)
            cv2.imshow('Color Image', color_image)
            # cv2.imshow('Edges', edges)
            cv2.imwrite(dataset_path + "processed/canny/" + f'{str(idx + 1).zfill(6)}.png', edges)
            cv2.imshow('Filtered Image with Largest Contour', filtered_img)
            cv2.imwrite(dataset_path + "processed/polygon_extracted/" + f'{str(idx + 1).zfill(6)}.png', filtered_img)
            print('Loop time cost', 1000 * (time.time() - time_start_loop), 'ms')

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
