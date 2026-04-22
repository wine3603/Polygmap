#!/usr/bin/env python3
from time import sleep
import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPoses  # 导入自定义消息类型
import numpy as np
# from utils.sat import RotatingRectangle

class StairClimbingPlanner:
    def __init__(self):
        error=0.0
        self.dt = 1  # 每一步的时间间隔（单位：秒）
        self.foot_width = 0.10  # 双脚之间的横向宽度（左右脚的y轴偏移）
        self.step_height =0.204#+0.096# 台阶高度（上楼/下楼的z轴变化）
        self.step_length = 0.386 #+ 0.02# 台阶长度（x方向的移动距离）
        self.total_step = 0  # 当前累计的步数（用于生成时间轨迹）

    def move_to_down_stairs(self, step=3, current_torso_pos=np.array([0.0, 0.0, 0.0, 0.0]), current_foot_pos=np.array([0.0, 0.0, 0.0])):
        """
        躯干位置调整到楼梯下方起始点的位置，使脚掌在正确位置准备下楼。
        """
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []

        step_length = 0.11  # 下楼前的移动步长较短
        for i in range(step):
            self.total_step += 1
            time_traj.append(self.total_step * self.dt)

            is_left_foot = ((self.total_step - 1) % 2 == 0)
            foot_idx_traj.append(0 if is_left_foot else 1)

            if i == 0:
                current_torso_pos[0] += step_length / 2
                current_foot_pos[0] = current_torso_pos[0] + step_length / 2
            elif i == step - 1:
                current_torso_pos[0] += step_length / 2
                current_foot_pos[0] = current_torso_pos[0]
            else:
                current_torso_pos[0] += step_length
                current_foot_pos[0] = current_torso_pos[0] + step_length / 2

            foot_traj.append([current_foot_pos[0], self.foot_width if is_left_foot else -self.foot_width, current_foot_pos[2], 0])
            torso_traj.append([*current_torso_pos, 0.0])

        return time_traj, foot_idx_traj, foot_traj, torso_traj

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
            if i == 2:
                print("foot_traj[i]:", foot_traj[i])
                print("foot_traj[i-1]:", foot_traj[i-1])
                for xx in range(7):
                    step_fp = footPose()
                    # 对x,y进行线性插值
                    print( np.array(np.array(foot_traj[i]) + np.array(foot_traj[i-1])*-1))
                    step_fp.footPose = foot_traj[i-1] + (np.array(foot_traj[i]) - np.array(foot_traj[i-1])) * (xx / 6)
                    # 对z方向添加抬脚高度轨迹，使用正弦函数实现平滑的抬升和下降
                    step_fp.footPose[2] = 0.2 * np.sin(np.pi * xx / 6)  # 最大高度0.10米
                    step_fp.footPose[1] = foot_traj[i][1] 
                    # 只对yaw角度进行特殊处理，使用正弦函数实现0-30度的来回摆动
                    yaw_angle = 15 * np.sin(np.pi * xx / 3)  # 15度是30度的一半，实现0-30度的范围
                    step_fp.footPose[3] = yaw_angle  # 修改yaw角度
                    print("step_fp:", step_fp.footPose)
                    additionalFootPoseTrajectory.data.append(step_fp)
            msg.additionalFootPoseTrajectory.append(additionalFootPoseTrajectory)
            # 将脚姿态添加到消息中
            msg.footPoseTrajectory.append(foot_pose_msg)

        return msg

    def plan_up_stairs(self, num_steps=2, current_torso_pos=np.array([0.0, 0.0, 0.0]), current_foot_pos=np.array([0.0, 0.0, 0.0])):
        """
        规划上楼梯的步态轨迹
        """
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []

        torso_height_offset = -0.0  # 上楼前略降低躯干高度
        current_torso_pos[2] = torso_height_offset
        torso_yaw = 0.0
        offset_x = [0.0, -0.0, -0.0, -0.0, -0.0]

        current_torso_pos[0]=0
        self.total_step=0

        for step in range(num_steps):
            self.total_step += 1
            time_traj.append(self.total_step * self.dt)

            is_left_foot = ((self.total_step - 1) % 2 == 0)
            foot_idx_traj.append(0 if is_left_foot else 1)

            if step%2==0 :
                current_foot_pos[0] = current_torso_pos[0] + self.step_length
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width
                current_foot_pos[2] = self.step_height
                current_torso_pos[0] = self.step_length * 0.05
            else:
                current_torso_pos[0] += self.step_length * 0.95
                # current_torso_pos[2] += self.step_height
                current_torso_pos[2] += 0.1
                current_foot_pos[0] = current_foot_pos[0] * 0.9
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width
                current_foot_pos[2] = self.step_height+0.015

            if step < len(offset_x) and step != num_steps - 1:
                current_foot_pos[0] += offset_x[step]

            foot_traj.append([*current_foot_pos, torso_yaw])
            torso_traj.append([*current_torso_pos, torso_yaw])

        return time_traj, foot_idx_traj, foot_traj, torso_traj

    def plan_down_stairs(self, num_steps=5, current_torso_pos=np.array([0.0, 0.0, 0.0]), current_foot_pos=np.array([0.0, 0.0, 0.0])):
        """
        规划下楼梯的步态轨迹
        """
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []

        torso_height_offset = -0.0  # 初始时不改变高度
        current_torso_pos[2] += torso_height_offset
        torso_yaw = 0.0
        offset_x = [0.0, -0.04, -0.115, -0.16, -0.0]
        first_step_offset = 0.328

        for step in range(num_steps):
            self.total_step += 1
            time_traj.append(self.total_step * self.dt)

            is_left_foot = ((self.total_step - 1) % 2 == 0)
            foot_idx_traj.append(0 if is_left_foot else 1)

            if step == 0:
                current_torso_pos[0] += first_step_offset
                current_foot_pos[0] = current_torso_pos[0]
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width
                current_foot_pos[2] -= self.step_height
                current_torso_pos[2] -= self.step_height
            elif step == num_steps - 1:
                current_torso_pos[0] = current_foot_pos[0]
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width
                current_torso_pos[2] += self.step_height
            else:
                current_torso_pos[0] += self.step_length
                current_torso_pos[2] -= self.step_height
                current_foot_pos[0] = current_torso_pos[0]
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width
                current_foot_pos[2] -= self.step_height

            if step < len(offset_x) and step != num_steps - 1:
                current_foot_pos[0] += offset_x[step]

            foot_traj.append([*current_foot_pos, torso_yaw])
            torso_traj.append([*current_torso_pos, torso_yaw])

        return time_traj, foot_idx_traj, foot_traj, torso_traj

def publish_foot_pose_traj(time_traj, foot_idx_traj, foot_traj, torso_traj):
    """
    发布 footPoseTargetTrajectories 消息给 humanoid 控制器
    """
    rospy.init_node('stair_climbing_planner', anonymous=True)
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)
    rospy.sleep(1)

    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = foot_idx_traj
    msg.footPoseTrajectory = []

    for i in range(len(time_traj)):
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]
        foot_pose_msg.torsoPose = torso_traj[i]
        msg.footPoseTrajectory.append(foot_pose_msg)

    pub.publish(msg)
    rospy.sleep(1.5)

if __name__ == '__main__':
    try:
        planner = StairClimbingPlanner()


        for i in range(1):  # 上几级
            print(f"\n--- Planning iteration {i + 1} ---")
            
            # 上楼
            time_traj_0, foot_idx_traj_0, foot_traj_0, torso_traj_0 = planner.plan_up_stairs()
            print("Up stairs plan done.")
            
            # 轨迹赋值
            time_traj = time_traj_0 
            foot_idx_traj = foot_idx_traj_0 
            foot_traj = foot_traj_0 
            torso_traj = torso_traj_0 
            
            # 输出结果
            print("Full stair walking trajectory planned.")
            print("Time trajectory:", time_traj)
            print("Foot index trajectory:", foot_idx_traj)
            print("Foot pose trajectory:", foot_traj)
            print("Torso pose trajectory:", torso_traj)
            
            # 发布轨迹
            publish_foot_pose_traj(time_traj, foot_idx_traj, foot_traj, torso_traj)
            
            # 暂停
            sleep(6)



    except rospy.ROSInterruptException:
        pass
