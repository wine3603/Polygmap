#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPoses  # 导入自定义消息类型
import numpy as np
from utils.sat import RotatingRectangle


import numpy as np

def euler_to_rotation_matrix(yaw, pitch, roll):
    # 计算各轴的旋转矩阵
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

    # 按照 Yaw-Pitch-Roll 的顺序组合旋转矩阵
    R = np.dot(R_roll, np.dot(R_pitch, R_yaw))
    return R


def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    num = len(time_traj)

    # 创建消息实例
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj  # 设置时间轨迹
    msg.footIndexTrajectory = foot_idx_traj         # 设置脚索引
    msg.footPoseTrajectory = []  # 初始化脚姿态轨迹

    for i in range(num):
        # 创建脚姿态信息
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]      # 设置脚姿态
        foot_pose_msg.torsoPose = torso_traj[i]    # 设置躯干姿态
        additionalFootPoseTrajectory = footPoses()
        foot_h=2*0.075#抬腿高度
        if i == 0:
            print("foot_traj[i]:", foot_traj[i])
            print("foot_traj[i+1]:", foot_traj[i+1])
            for xx in range(7):
                step_fp = footPose()
                # 对x,y进行线性插值
                print( np.array(foot_traj[i]) * (xx / 6))
                step_fp.footPose =  np.array(foot_traj[i]) * (xx / 6)
                # 对z方向添加抬脚高度轨迹，使用正弦函数实现平滑的抬升和下降
                step_fp.footPose[2] = foot_h * np.sin(np.pi * xx / 6)  # 最大高度0.10米
                step_fp.footPose[1] = foot_traj[i][1] 
                # 只对yaw角度进行特殊处理，使用正弦函数实现0-30度的来回摆动
                # yaw_angle = 15 * np.sin(np.pi * xx / 3)  # 15度是30度的一半，实现0-30度的范围
                # step_fp.footPose[3] = yaw_angle  # 修改yaw角度
                # print("step_fp:", step_fp.footPose)
                additionalFootPoseTrajectory.data.append(step_fp)
        if i == 1:
            for xx in range(7):
                step_fp = footPose()
                # 对x,y进行线性插值
                print( np.array(foot_traj[i]) * (xx / 6))
                step_fp.footPose =  np.array(foot_traj[i]) * (xx / 6)
                # 对z方向添加抬脚高度轨迹，使用正弦函数实现平滑的抬升和下降
                step_fp.footPose[2] = foot_h * np.sin(np.pi * xx / 6)  # 最大高度0.10米
                step_fp.footPose[1] = foot_traj[i][1] 
                # 只对yaw角度进行特殊处理，使用正弦函数实现0-30度的来回摆动
                # yaw_angle = 15 * np.sin(np.pi * xx / 3)  # 15度是30度的一半，实现0-30度的范围
                # step_fp.footPose[3] = yaw_angle  # 修改yaw角度
                # print("step_fp:", step_fp.footPose)
                additionalFootPoseTrajectory.data.append(step_fp)
        msg.additionalFootPoseTrajectory.append(additionalFootPoseTrajectory)
        # 将脚姿态添加到消息中
        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg

def generate_steps(torso_pos, torso_yaw, foot_bias):
    # 构造左脚偏移向量：在 Y 轴正方向上偏移 foot_bias，高度设为 -torso_pos[2]（使脚在地面 z=0）
    l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])

    # 构造右脚偏移向量：在 Y 轴负方向上偏移 foot_bias，高度设为 -torso_pos[2]（使脚在地面 z=0）
    r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])

    # 构造绕 Z 轴旋转的 3x3 旋转矩阵，用于将局部坐标转换为世界坐标
    R_z = np.array([
        [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
        [np.sin(torso_yaw),  np.cos(torso_yaw), 0],
        [0,                 0,                  1]
    ])

    # 左脚在世界坐标中的位置 = 躯干位置 + 躯干旋转后的偏移向量
    l_foot = torso_pos + R_z.dot(l_foot_bias)

    # 右脚在世界坐标中的位置 = 躯干位置 + 躯干旋转后的偏移向量
    r_foot = torso_pos + R_z.dot(r_foot_bias)

    # 返回左右脚的位置（三维向量）
    return l_foot, r_foot


def get_multiple_steps_msg(body_poses, dt, is_left_first=True, collision_check=True):
    # 步数为身体姿态数量的两倍（左右脚各走一步）
    num_steps = 2 * len(body_poses)

    # 初始化时间轨迹、足部编号轨迹（0左脚，1右脚）、足部轨迹和躯干轨迹
    time_traj = []
    foot_idx_traj = []
    foot_traj = []
    torso_traj = []

    # 初始左脚和右脚的碰撞矩形框（假设脚宽0.24米，脚长0.1米）
    l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
    r_foot_rect_last = RotatingRectangle(center=(0,-0.1), width=0.24, height=0.1, angle=0)

    # 初始躯干朝向角（yaw）和上一次躯干姿态（位置 + yaw）
    torso_yaw_last = 0.0
    torso_pose_last = np.array([0, 0, 0, 0])

    # 遍历每一步
    for i in range(num_steps):
        # 添加该步的时间点
        time_traj.append(dt * (i + 1))

        # 获取当前对应的躯干姿态（每两步共享一个）
        body_pose = body_poses[i // 2]
        torso_pos = np.asarray(body_pose[:3])  # x, y, z
        torso_yaw = np.radians(body_pose[3])   # 将yaw角度转换为弧度

        # 基于当前躯干位置和方向，生成左右脚的位置（间距0.1米）
        l_foot, r_foot = generate_steps(torso_pos, torso_yaw, 0.1)

        # 添加躯干朝向角到左右脚位姿
        l_foot = [*l_foot[:3], torso_yaw]
        r_foot = [*r_foot[:3], torso_yaw]
        l_foot[2]=torso_pos[2]
        r_foot[2]=torso_pos[2]

        # 每两个步伐的第一个，用来判断迈哪只脚
        if i % 2 == 0:
            # 当前躯干姿态
            torso_pose = np.array([*body_pose[:3], torso_yaw])

            # 使用上一帧的朝向计算局部坐标变换
            R_wl = euler_to_rotation_matrix(torso_pose_last[3], 0, 0)
            delta_pos = R_wl.T @ (torso_pose[:3] - torso_pose_last[:3])  # 变换到局部坐标系

            print("delta_pos:", delta_pos)

            # 如果朝向偏左或偏左移动，则优先迈左脚，否则右脚
            if torso_yaw > 0.0 or delta_pos[1] > 0.0:
                is_left_first = True
            else:
                is_left_first = False

        # 检查脚步间是否发生碰撞（仅对每两个步伐的第一个进行）
        if collision_check and i % 2 == 0:
            # 生成下一步左右脚的碰撞矩形
            l_foot_rect_next = RotatingRectangle(center=(l_foot[0], l_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            r_foot_rect_next = RotatingRectangle(center=(r_foot[0], r_foot[1]), width=0.24, height=0.1, angle=torso_yaw)

            # 检查是否与上一步对侧脚碰撞
            l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
            r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)

            # 若左右脚都碰撞，终止并提示错误
            if l_collision and r_collision:
                print("\033[91m[Error] Detect collision, Please adjust your body_poses input!!!\033[0m")
                break
            # 左脚碰撞，改为右脚先迈
            elif l_collision:
                print("\033[92m[Info] Left foot is in collision, switch to right foot\033[0m")
                is_left_first = False
            # 右脚碰撞，改为左脚先迈
            elif r_collision:
                print("\033[92m[Info] Right foot is in collision, switch to left foot\033[0m")
                is_left_first = True

            # 更新上一帧的脚底状态
            l_foot_rect_last = l_foot_rect_next
            r_foot_rect_last = r_foot_rect_next

        # 构造每一步的躯干轨迹和足部轨迹
        if i % 2 == 0:
            # 第一步时用当前与上一躯干中点插值作为躯干轨迹
            torso_traj.append((torso_pose_last + torso_pose) / 2.0)

            # 添加脚步索引和脚步位姿
            if is_left_first:
                foot_idx_traj.append(0)     # 左脚
                foot_traj.append(l_foot)
            else:
                foot_idx_traj.append(1)     # 右脚
                foot_traj.append(r_foot)
        else:
            # 第二步用当前躯干姿态
            torso_traj.append(torso_pose)

            # 添加脚步信息（另一只脚）
            if is_left_first:
                foot_idx_traj.append(1)     # 右脚
                foot_traj.append(r_foot)
            else:
                foot_idx_traj.append(0)     # 左脚
                foot_traj.append(l_foot)

        # 更新躯干状态
        torso_pose_last = torso_traj[-1]
        torso_yaw_last = torso_yaw

    # 打印结果轨迹
    print("time_traj:", time_traj)
    print("foot_idx_traj:", foot_idx_traj)
    print("foot_traj:", foot_traj)
    print("torso_traj:", torso_traj)

    # 返回组合的轨迹消息
    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)



if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('foot_pose_publisher', anonymous=True)
    # 创建发布者，话题为 /humanoid_mpc_foot_pose_target_trajectories
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

    # 等待一定时间以确保订阅者已经准备好
    rospy.sleep(1)

    is_left_first_default = True # 缺省左脚先行
    # 缺省开启碰撞检测，如果默认规划的步态顺序会导致碰撞，则会自动切换到另一侧的步态,如果设置为False则不会切换步态顺序
    # 注意：碰撞检测开启后，并且可能导致规划失败
    collision_check = True 
    # body_poses基于局部坐标系给定，每一个身体姿态对应两步到达
    dt = 0.5 #迈一步的时间间隔，腾空相和支撑相时间占比各dt/2
    # 一次完整的步态Mode序列为:[SS FS SS SF SS]或者[SS SF SS FS SS]
    # body_pose： [x(m), y(m), z(m), yaw(deg)]
    body_poses = [
        # [0.15, 0.1, 0, 25],

        [0.1, 0.0, 0, 0],
        [0.2, 0.0, 0, 0],
        [0.3, 0.0, 0, 0],
        [0.4, 0.0, 0, 0],
        [0.55, 0.0, 0, 35],
        # [0.7, 0.0, 0, 0],

        # [0.2, -0.1, 0, -30],
        # [0.3, 0.0, 0, -0],
        # [0.4, 0.0, 0, -30],
        # [0.5, 0.0, 0, 0],
    ]
    msg = get_multiple_steps_msg(body_poses, dt, is_left_first_default, collision_check)
    pub.publish(msg)
