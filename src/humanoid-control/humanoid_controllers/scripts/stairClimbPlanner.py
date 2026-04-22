#!/usr/bin/env python3

import rospy
import numpy as np
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPoses
from scipy.interpolate import CubicSpline, PchipInterpolator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Stair climbing planner')
    parser.add_argument('--plot', action='store_true', help='Enable trajectory plotting')
    parser.add_argument('--stand-height', type=float, default=0.0, help='Stand height offset (default: 0.0)')
    return parser.parse_args()

args = parse_args()
PLOT = args.plot
STAND_HEIGHT = args.stand_height

class StairClimbingPlanner:
    def __init__(self):
        self.dt = 0.8  # 步态周期
        self.foot_width = 0.10  # 宽
        self.step_height = 0.1  # 台阶高度
        self.step_length = 0.315  # 台阶长度
        self.total_step = 0  # 总步数
        
    def move_to_down_stairs(self, step = 3, current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0]), current_foot_pos = np.array([0.0, 0.0, 0.0])):
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []
        
        # current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0])
        # current_foot_pos = np.array([0.0, 0.0, 0.0])
        step_length = 0.11
        for i in range(step):
            self.total_step += 1
            time_traj.append(self.total_step*self.dt)
            # 左右脚交替
            is_left_foot = ((self.total_step -1) % 2 == 0)
            foot_idx_traj.append(0 if is_left_foot else 1)
            if i == 0:
                current_torso_pos[0] += step_length/2
                current_foot_pos[0] = current_torso_pos[0] +step_length/2  # 脚掌相对躯干前移
            elif i == step - 1:
                current_torso_pos[0] += step_length/2
                current_foot_pos[0] = current_torso_pos[0]
            else:
                current_torso_pos[0] += step_length
                current_foot_pos[0] = current_torso_pos[0] +step_length/2  # 脚掌相对躯干前移
            
            foot_traj.append([current_foot_pos[0],self.foot_width if is_left_foot else -self.foot_width, current_foot_pos[2],0])
            torso_traj.append([*current_torso_pos, 0.0])
            
        return time_traj, foot_idx_traj, foot_traj, torso_traj
    
    
    def plan_up_stairs(self, num_steps=6, current_torso_pos = np.array([0.0, 0.0, 0.0]), current_foot_pos = np.array([0.0, 0.0, 0.0])):
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []
        swing_trajectories = []  # 存储腾空相轨迹
        
        # 初始位置
        torso_height_offset = -0.1  # 躯干高度偏移
        current_torso_pos[2] = torso_height_offset
        torso_yaw = 0.0
        # current_foot_pos = np.array([0.0, 0.0, 0.0])
        offset_x = [0.0, -0.0, -0.0, -0.0, -0.0]
        first_step_offset = 0.35
        
        # 记录前一次的左右脚位置
        prev_left_foot = [0, 0.1, STAND_HEIGHT, 0.0]
        prev_right_foot = [0, -0.1, STAND_HEIGHT, 0.0]
        
        # 为每一步生成落脚点
        for step in range(num_steps):
            # 更新时间
            self.total_step += 1
            time_traj.append((0 if step == 0 else time_traj[-1]) + self.dt)
            
            # 左右脚交替
            is_left_foot = ((self.total_step -1) % 2 == 0)
            foot_idx_traj.append(0 if is_left_foot else 1)
            
            # 计算躯干位置
            if step == 0:
                
                current_foot_pos[0] = current_torso_pos[0] + self.step_length  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] = self.step_height + STAND_HEIGHT  # 脚掌高度
                current_torso_pos[0] += self.step_length/3
                
            elif step == num_steps - 1: # 最后一步
                # current_torso_pos[0] += self.step_length/2  # 向前移动
                # current_torso_pos[2] += self.step_height/2  # 向上移动
                # current_foot_pos[0] = current_torso_pos[0] # 最后一步x不动
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width  # 左右偏移
                current_torso_pos[2] += self.step_height 
            else:
                current_torso_pos[0] += self.step_length  # 向前移动
                current_torso_pos[2] += self.step_height  # 向上移动
            
                # 计算落脚点位置
                current_foot_pos[0] = current_torso_pos[0] + self.step_length/2  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] += self.step_height
                
            if step < len(offset_x) and not step == num_steps - 1:    # 脚掌偏移
                current_foot_pos[0] += offset_x[step]
                
            # 记录当前脚的位置
            current_foot = [*current_foot_pos, torso_yaw]
            
            # 生成腾空相轨迹
            if prev_left_foot is not None and prev_right_foot is not None:  # 从第二步开始生成腾空相轨迹
                prev_foot = prev_left_foot if is_left_foot else prev_right_foot
                swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.1, plot=PLOT)
                swing_trajectories.append(swing_traj)
            
            # 更新前一次的脚位置
            if is_left_foot:
                prev_left_foot = current_foot
            else:
                prev_right_foot = current_foot
            
            # 添加轨迹点
            foot_traj.append(current_foot)
            torso_traj.append([*current_torso_pos, torso_yaw])
            
            last_torso_pose = torso_traj[-1].copy()
            last_foot_pose = foot_traj[-1].copy()
            # add SS 
            if step != num_steps - 1:
                
                time_traj.append(time_traj[-1] + 0.4)
                foot_idx_traj.append(2)
                foot_traj.append([0.0, 0.0, 0.0, 0.0])
                last_torso_pose[0] = last_foot_pose[0]
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + 0.4)
                foot_idx_traj.append(2)
                foot_traj.append([0.0, 0.0, 0.0, 0.0])
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
            
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

        
    def plan_down_stairs(self, num_steps=5, current_torso_pos = np.array([0.0, 0.0, 0.0]), current_foot_pos = np.array([0.0, 0.0, 0.0])):
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []
        
        # 初始位置
        torso_height_offset = -0.0  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        torso_yaw = 0.0
        # current_foot_pos = np.array([0.0, 0.0, 0.0])
        offset_x = [0.0, -0.0, -0.0, -0.0, -0.0]
        
        first_step_offset = 0.328
        
        # 为每一步生成落脚点
        for step in range(num_steps):
            # 更新时间
            self.total_step += 1
            time_traj.append(self.total_step * self.dt)
            
            # 左右脚交替
            is_left_foot = ((self.total_step -1) % 2 == 0)
            foot_idx_traj.append(0 if is_left_foot else 1)
            
            # 计算躯干位置
            if step == 0:
                current_torso_pos[0] += first_step_offset
                current_foot_pos[0] = current_torso_pos[0]  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] -= self.step_height  # 脚掌高度
                current_torso_pos[2] -= self.step_height  # 脚掌高度
            elif step == num_steps - 1: # 最后一步
                # current_torso_pos[0] += self.step_length/2  # 向前移动
                # current_torso_pos[2] += self.step_height/2  # 向上移动
                # current_foot_pos[0] = current_torso_pos[0] # 最后一步x不动
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width  # 左右偏移
                current_torso_pos[2] += self.step_height  # 脚掌高度
            else:
                current_torso_pos[0] += self.step_length  # 向前移动
                current_torso_pos[2] -= self.step_height  # 向上移动
            
                # 计算落脚点位置
                current_foot_pos[0] = current_torso_pos[0]  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] -= self.step_height
                
            if step < len(offset_x) and not step == num_steps - 1:    # 脚掌偏移
                current_foot_pos[0] += offset_x[step]
                # current_torso_pos[0] += offset_x[step]
            # 添加轨迹点
            foot_traj.append([*current_foot_pos, torso_yaw])
            torso_traj.append([*current_torso_pos, torso_yaw])
            
        return time_traj, foot_idx_traj, foot_traj, torso_traj

    def plan_swing_phase(self, prev_foot_pose, next_foot_pose, swing_height=0.10, plot=False):
        """
        使用形状保持的三次样条插值规划腾空相的轨迹
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            swing_height: 抬脚最大高度，默认0.2米
            plot: 是否绘制轨迹图，默认False
        Returns:
            additionalFootPoseTrajectory: 包含腾空相轨迹的footPoses消息
        """
        additionalFootPoseTrajectory = footPoses()
        num_points = 7  # 轨迹点数量
        
        # 创建时间序列
        t = np.linspace(0, 1, num_points)
        
        # 计算x和y方向的移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        
        # 计算基准高度（取两个落点中较高的点）
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        
        # 创建控制点
        # 时间点：0, 0.2, 0.5, 1.0
        # 0.2时刻：x和y移动10%，z达到最高点
        # 0.5时刻：x和y移动50%，z保持最高点
        control_points = {
            't': [0, 0.2, 0.5, 1.0],
            'x': [
                prev_foot_pose[0],                    # 起点
                prev_foot_pose[0] + x_distance * 0.10, # 前10%
                prev_foot_pose[0] + x_distance * 0.5, # 前50%
                next_foot_pose[0]                     # 终点
            ],
            'y': [
                prev_foot_pose[1],                    # 起点
                prev_foot_pose[1] + y_distance * 0.10, # 前10%
                prev_foot_pose[1] + y_distance * 0.5, # 前50%
                next_foot_pose[1]                     # 终点
            ],
            'z': [
                prev_foot_pose[2],                    # 起点
                base_height + swing_height*0.6,           # 最高点（基于较高的落点）
                base_height + swing_height,           # 保持最高点
                next_foot_pose[2]                     # 终点
            ]
        }
        
        # 为x、y和z创建形状保持的三次样条插值
        x_spline = PchipInterpolator(control_points['t'], control_points['x'])
        y_spline = PchipInterpolator(control_points['t'], control_points['y'])
        z_spline = PchipInterpolator(control_points['t'], control_points['z'])
        
        # yaw角度使用形状保持的三次样条
        yaw_spline = PchipInterpolator([0, 1], [prev_foot_pose[3], next_foot_pose[3]])
        
        # 生成轨迹点
        trajectory_points = []
        for i in range(num_points):
            step_fp = footPose()
            x = float(x_spline(t[i]))
            y = float(y_spline(t[i]))
            z = float(z_spline(t[i]))
            yaw = float(yaw_spline(t[i]))
            
            step_fp.footPose = [x, y, z, yaw]
            additionalFootPoseTrajectory.data.append(step_fp)
            trajectory_points.append([x, y, z])
            
        # 如果需要绘图
        if plot:
            # 创建更密集的时间序列用于绘制平滑曲线
            t_dense = np.linspace(0, 1, 100)
            x_dense = x_spline(t_dense)
            y_dense = y_spline(t_dense)
            z_dense = z_spline(t_dense)
            
            # 创建3D图
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # 绘制轨迹
            ax.plot(x_dense, y_dense, z_dense, 'b-', label='trajectory')
            
            # 绘制控制点
            ax.scatter(control_points['x'], control_points['y'], control_points['z'], 
                      c='r', marker='o', label='control points')
            
            # 绘制实际轨迹点
            trajectory_points = np.array(trajectory_points)
            ax.scatter(trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2],
                      c='g', marker='^', label='trajectory points')
            
            # 设置图表属性
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title('foot pose trajectory')
            
            # 添加图例
            ax.legend()
            
            # 设置坐标轴比例相等
            max_range = np.array([
                max(x_dense) - min(x_dense),
                max(y_dense) - min(y_dense),
                max(z_dense) - min(z_dense)
            ]).max() / 2.0
            
            mid_x = (max(x_dense) + min(x_dense)) * 0.5
            mid_y = (max(y_dense) + min(y_dense)) * 0.5
            mid_z = (max(z_dense) + min(z_dense)) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            # 显示图形
            plt.show()
            
        return additionalFootPoseTrajectory

def publish_foot_pose_traj(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories=None):
    rospy.init_node('stair_climbing_planner', anonymous=True)
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', 
                         footPoseTargetTrajectories, queue_size=10)
    rospy.sleep(1)

    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = foot_idx_traj
    msg.footPoseTrajectory = []
    msg.additionalFootPoseTrajectory = []

    for i in range(len(time_traj)):
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]
        foot_pose_msg.torsoPose = torso_traj[i]
        msg.footPoseTrajectory.append(foot_pose_msg)
        
        # 如果有腾空相轨迹，添加到消息中
        if swing_trajectories is not None and i < len(swing_trajectories):
            swing_poses = footPoses()
            # 将swing_trajectories[i]中的轨迹点添加到swing_poses中
            # for pose in swing_trajectories[i]:
            #     swing_poses.data.append(pose)
            msg.additionalFootPoseTrajectory.append(swing_trajectories[i])
        else:
            msg.additionalFootPoseTrajectory.append(footPoses())  # 添加空的轨迹

    pub.publish(msg)
    rospy.sleep(1.5)

if __name__ == '__main__':
    try:
        planner = StairClimbingPlanner()
        time_traj_0, foot_idx_traj_0, foot_traj_0, torso_traj_0, swing_trajectories_0 = planner.plan_up_stairs()
        print("Up stairs plan done.")
        print("Time trajectory:", time_traj_0)
        print("Foot index trajectory:", foot_idx_traj_0)
        print("Foot pose trajectory:", foot_traj_0)
        print("Torso pose trajectory:", torso_traj_0)
        print("Number of swing trajectories:", len(swing_trajectories_0))
        
        time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = time_traj_0, foot_idx_traj_0, foot_traj_0, torso_traj_0, swing_trajectories_0
        # time_traj_1, foot_idx_traj_1, foot_traj_1, torso_traj_1 = planner.move_to_down_stairs(current_torso_pos=np.array(torso_traj_0[-1][0:3]), current_foot_pos=np.array(foot_traj_0[-1][0:3]))
        # print("\nMove to down stairs plan done.")
        # print("Time trajectory:", time_traj_1)
        # print("Foot index trajectory:", foot_idx_traj_1)
        # print("Foot pose trajectory:", foot_traj_1)
        # print("Torso pose trajectory:", torso_traj_1)
        # time_traj += time_traj_1
        # foot_idx_traj += foot_idx_traj_1
        # foot_traj += foot_traj_1
        # torso_traj += torso_traj_1
        # print(torso_traj_1[-1][0:3])
        # time_traj_2, foot_idx_traj_2, foot_traj_2, torso_traj_2 = planner.plan_down_stairs(current_torso_pos=np.array(torso_traj_1[-1][0:3]), current_foot_pos=np.array(foot_traj_1[-1][0:3]))
        # print("\nDown stairs plan done.")
        # print("Time trajectory:", time_traj_2)
        # print("Foot index trajectory:", foot_idx_traj_2)
        # print("Foot pose trajectory:", foot_traj_2)
        # print("Torso pose trajectory:", torso_traj_2)
        # time_traj += time_traj_2
        # foot_idx_traj += foot_idx_traj_2
        # foot_traj += foot_traj_2
        # torso_traj += torso_traj_2
        # 打印规划结果
        print("\nTime trajectory:", time_traj)
        print("Foot index trajectory:", foot_idx_traj)
        print("Foot pose trajectory:", foot_traj)
        print("Torso pose trajectory:", torso_traj)
        
        publish_foot_pose_traj(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
    except rospy.ROSInterruptException:
        pass
