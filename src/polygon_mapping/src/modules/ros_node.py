#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
import tf.transformations  
import copy

from polygon_mapping.msg import Polygon3D as Polygon3D_msg
from polygon_mapping.msg import MapManager as MapManager_msg
from geometry_msgs.msg import Point32, Vector3

from visualization_msgs.msg import Marker
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans

############################################################### Listen to TF and publish map ########################################################
class MappingNode:
    def __init__(self, map_manager, map_frame='world', camera_frame='camera2_depth', frequency=30.0):
        # Initialize the ROS node
        rospy.init_node('polygon_node', anonymous=True)

        self.T = np.eye(4)

        # Create TF buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.map_frame = map_frame
        self.camera_frame = camera_frame

        # Set callback function for transformations
        self.set_transform_callback()

        # Connect MapManager to ROS
        self.map_manager = map_manager

        self.frequency = frequency
        self.pub = rospy.Publisher('/map_manager', MapManager_msg, queue_size=5)

        # Create a publisher for visualization markers
        self.previous_marker_count = 0  # Track the previous number of markers published
        # self.marker_pub = rospy.Publisher('/polygon_markers', Marker, queue_size=20)
        self.marker_pub = rospy.Publisher('/polygon_markers', MarkerArray, queue_size=10)

        # Initialize pose publishers
        self.pose_pub = rospy.Publisher('/camera_pose', PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/camera_path', Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "odom"

        # Raw (uncompensated) pose publisher
        self.pose_raw_pub = rospy.Publisher('/camera_pose_raw', PoseStamped, queue_size=1)
        self.path_raw_pub = rospy.Publisher('/camera_raw_path', Path, queue_size=10)
        self.path_raw = Path()
        self.path_raw.header.frame_id = "odom"

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.publish_polygons)

    def set_transform_callback(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.transform_callback)

    def transform_callback(self, event):
        try:
            # Retrieve the transform from /camera2_link to /world
            trans = self.tfBuffer.lookup_transform(self.map_frame, self.camera_frame, rospy.Time(0))
            self.T = self.transform_to_matrix(trans)
            # rospy.loginfo("Transform from /camera2_link to /world: \n%s" % T)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")

    def transform_to_matrix(self, trans):
        # Convert TransformStamped to a 4x4 transformation matrix
        translation = trans.transform.translation
        rotation = trans.transform.rotation

        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        T = tf.transformations.quaternion_matrix(q)
        T[0, 3] = translation.x
        T[1, 3] = translation.y
        T[2, 3] = translation.z

        return T

    def publish_polygons(self, event):
        # Publish the global map
        msg = MapManager_msg()
        for polygon in self.map_manager.polygons:
            poly_msg = Polygon3D_msg()
            for vertex in polygon.vertices:
                point = Point32()
                point.x, point.y, point.z = vertex
                poly_msg.vertices.append(point)

            normal = Vector3()
            normal.x, normal.y, normal.z = polygon.normal
            poly_msg.normal = normal

            poly_msg.plane_eq = polygon.plane_eq
            poly_msg.area = polygon.area

            msg.polygons.append(poly_msg)
        self.pub.publish(msg)

        T_lock = copy.deepcopy(self.T)
        self.publish_camera_pose(T_lock)

        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, polygon in enumerate(self.map_manager.polygons):
            marker = Marker()
            marker.header.frame_id = "odom"
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
                # 闭合多边形
                point = Point()
                point.x = polygon.vertices[0][0]
                point.y = polygon.vertices[0][1]
                point.z = polygon.vertices[0][2]
                marker.points.append(point)

            marker_array.markers.append(marker)

        # 一次性发布一组 marker
        self.marker_pub.publish(marker_array)

        # Assume self.map_manager.polygons is updated
        # for i, polygon in enumerate(self.map_manager.polygons):
        #     marker = Marker()
        #     marker.header.frame_id = "odom"
        #     marker.header.stamp = rospy.Time.now()
        #     marker.ns = "polygons"
        #     marker.id = i
        #     marker.type = Marker.LINE_STRIP
        #     marker.action = Marker.ADD
        #     marker.lifetime = rospy.Duration(0.5)  # Marker automatically disappears after 0.5 seconds if not updated
        #     marker.pose.orientation.w = 1.0
        #     marker.scale.x = 0.05
        #     marker.color.r = 0.0
        #     marker.color.g = 1.0
        #     marker.color.b = 0.0
        #     marker.color.a = 1.0

        #     # Add polygon vertices
        #     for vertex in polygon.vertices:
        #         point = Point()
        #         point.x = vertex[0]
        #         point.y = vertex[1]
        #         point.z = vertex[2]
        #         marker.points.append(point)
        #     if len(polygon.vertices) > 2:
        #         point = Point()
        #         point.x = polygon.vertices[0][0]
        #         point.y = polygon.vertices[0][1]
        #         point.z = polygon.vertices[0][2]
        #         marker.points.append(point)  # Close the polygon

        #     self.marker_pub.publish(marker)

    def publish_camera_pose(self, T):
        # Extract translation vector
        translation = T[0:3, 3]

        # Extract rotation matrix and convert to quaternion, keeping yaw
        rotation_matrix = T[0:3, 0:3]
        euler_angles = tf_trans.euler_from_matrix(rotation_matrix, 'sxyz')
        yaw = euler_angles[2] + 3.14159 / 2

        # Set roll and pitch to 0, reconstruct quaternion
        quaternion = tf_trans.quaternion_from_euler(0, 0, yaw)

        # Apply drift compensation to Z axis
        translation[2] += self.map_manager.z_drift

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"

        # Set position
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]

        # Set orientation (quaternion)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Publish the camera pose
        self.pose_pub.publish(pose_msg)

        # Update the path
        self.path.poses.append(pose_msg)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

        pose_raw_msg = copy.deepcopy(pose_msg)
        pose_raw_msg.pose.position.z -= self.map_manager.z_drift
        self.pose_raw_pub.publish(pose_raw_msg)

        # Update the raw path
        self.path_raw.poses.append(pose_raw_msg)
        self.path_raw.header.stamp = rospy.Time.now()
        self.path_raw_pub.publish(self.path_raw)
