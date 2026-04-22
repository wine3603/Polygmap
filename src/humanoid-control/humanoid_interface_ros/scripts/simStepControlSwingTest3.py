#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPoses
import numpy as np
from utils.sat import RotatingRectangle
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

received_nearest_point = False
latest_nearest_point = None
foot_now_h = 0

def euler_to_rotation_matrix(yaw, pitch, roll):
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
    R = np.dot(R_roll, np.dot(R_pitch, R_yaw))
    return R

def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    num = len(time_traj)
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = foot_idx_traj
    msg.footPoseTrajectory = []

    for i in range(num):
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]
        foot_pose_msg.torsoPose = torso_traj[i]

        additionalFootPoseTrajectory = footPoses()
        foot_h = 0.1
        x_f = foot_traj[i][0] - 0.13 - 0.125 - 0.05
        z_f = foot_traj[i][2] + foot_h - foot_now_h 

        if i in [0, 1]:
            if i == 0:
                foot_pose_msg.torsoPose[2] = torso_traj[i][2] - 0.2 * foot_now_h
            else:
                foot_pose_msg.torsoPose[2] = torso_traj[i][2] - foot_now_h

            for xx in range(7):
                step_fp = footPose()
                if xx <= 3:
                    step_fp.footPose[2] = z_f * np.sin((np.pi / 2) * xx / 3)
                    step_fp.footPose[0] = x_f * (xx / 3) if i == 0 else 0.8 * x_f * (xx / 3)
                else:
                    step_fp.footPose[2] = foot_h * np.sin(np.pi/2 + (np.pi/2) * (xx-3) / 3) + foot_traj[i][2] - foot_now_h
                    step_fp.footPose[0] = (foot_traj[i][0] - x_f) * ((xx-3) / 3) + (x_f if i == 0 else x_f * 0.8)
                step_fp.footPose[1] = foot_traj[i][1]
                step_fp.footPose[2] += foot_now_h
                additionalFootPoseTrajectory.data.append(step_fp)

        msg.additionalFootPoseTrajectory.append(additionalFootPoseTrajectory)
        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg

def generate_steps(torso_pos, torso_yaw, foot_bias):
    l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])
    r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])
    R_z = np.array([
        [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
        [np.sin(torso_yaw),  np.cos(torso_yaw), 0],
        [0, 0, 1]
    ])
    l_foot = torso_pos + R_z.dot(l_foot_bias)
    r_foot = torso_pos + R_z.dot(r_foot_bias)
    return l_foot, r_foot

def get_multiple_steps_msg(body_poses, dt, is_left_first=True, collision_check=True):
    num_steps = 2 * len(body_poses)
    time_traj, foot_idx_traj, foot_traj, torso_traj = [], [], [], []

    l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
    r_foot_rect_last = RotatingRectangle(center=(0,-0.1), width=0.24, height=0.1, angle=0)

    torso_yaw_last = 0.0
    torso_pose_last = np.array([0, 0, 0, 0])

    for i in range(num_steps):
        time_traj.append(dt * (i + 1))
        body_pose = body_poses[i // 2]
        torso_pos = np.asarray(body_pose[:3])
        torso_yaw = np.radians(body_pose[3])
        l_foot, r_foot = generate_steps(torso_pos, torso_yaw, 0.1)
        l_foot = [*l_foot[:3], torso_yaw]
        r_foot = [*r_foot[:3], torso_yaw]
        l_foot[2] = torso_pos[2]
        r_foot[2] = torso_pos[2]

        if i % 2 == 0:
            torso_pose = np.array([*body_pose[:3], torso_yaw])
            R_wl = euler_to_rotation_matrix(torso_pose_last[3], 0, 0)
            delta_pos = R_wl.T @ (torso_pose[:3] - torso_pose_last[:3])
            if torso_yaw > 0.0 or delta_pos[1] > 0.0:
                is_left_first = True
            else:
                is_left_first = False

        if collision_check and i % 2 == 0:
            l_foot_rect_next = RotatingRectangle(center=(l_foot[0], l_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            r_foot_rect_next = RotatingRectangle(center=(r_foot[0], r_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
            r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)
            if l_collision and r_collision:
                rospy.logerr("Collision detected, adjust body_poses input!")
                break
            elif l_collision:
                is_left_first = False
            elif r_collision:
                is_left_first = True
            l_foot_rect_last = l_foot_rect_next
            r_foot_rect_last = r_foot_rect_next

        if i % 2 == 0:
            torso_traj.append((torso_pose_last + torso_pose) * 0.2)
            if is_left_first:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
            else:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
        else:
            torso_traj.append(torso_pose)
            if is_left_first:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
            else:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)

        torso_pose_last = torso_traj[-1]
        torso_yaw_last = torso_yaw

    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)

def nearest_point_odom_callback(msg: Odometry):
    global received_nearest_point, latest_nearest_point
    latest_nearest_point = msg
    received_nearest_point = True

def timer_callback(event):
    global received_nearest_point, latest_nearest_point, foot_now_h
    if not received_nearest_point or latest_nearest_point is None:
        return

    msg = latest_nearest_point
    orientation_q = msg.pose.pose.orientation
    quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    yaw = yaw * 180.0 / 3.14159

    foot_now_h = msg.twist.twist.linear.z
    body_poses = [[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw]]
    dt = 0.5
    traj_msg = get_multiple_steps_msg(body_poses, dt, True, True)

    pub.publish(traj_msg)
    rospy.loginfo("Published foot pose trajectory")

if __name__ == '__main__':
    rospy.init_node('foot_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)
    rospy.Subscriber('/nearest_point', Odometry, nearest_point_odom_callback)
    rospy.Timer(rospy.Duration(3.0), timer_callback)
    rospy.spin()
