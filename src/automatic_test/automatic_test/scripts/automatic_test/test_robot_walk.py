#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import pytest
import rospy
from std_msgs.msg import String
from enum import Enum
from trace_path.mpc_client_example import start_mpc_tracer, stop_mpc_tracer, back_to_home
from trace_path.mpc_path_tracer import FollowState
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerRequest
from .conftest import AutoTrackingStats

follow_state = FollowState.NOT_FOLLOWING


def follow_state_callback(msg):
    global follow_state
    follow_state = FollowState(msg.data)


@pytest.mark.usefixtures("ros_setup")
class TestRobotWalk:
    def setup_class(self):
        self.follow_state_sub = rospy.Subscriber('trace_path/follow_state', Int8, follow_state_callback)
        self.available_path_types = ['triangle', 'line', 'circle', 'square', 'scurve']
        self.ros_namespace = 'test_robot_walk'
    
    def wait_for_completion(self, check_robot_alive, auto_tracking_stats, description=""):
        while follow_state != FollowState.FINISHED:
            rospy.sleep(0.1)
            assert check_robot_alive.get_status(), f"机器人死亡: {check_robot_alive.get_error()}"
            assert auto_tracking_stats.status == AutoTrackingStats.FOLLOWING, \
                   f"动捕天轨系统自动跟踪机器人失败 ({description})"
    
    def execute_path(self, path_type, check_robot_alive, auto_tracking_stats):
        start_mpc_tracer(path_type)
        rospy.sleep(1.0)
        self.wait_for_completion(check_robot_alive, auto_tracking_stats, f"执行{path_type}路径")
        
        back_to_home()
        rospy.sleep(1.0)
        self.wait_for_completion(check_robot_alive, auto_tracking_stats, "返回家")
    
    @pytest.mark.walk
    def test_robot_walk(self, check_robot_ready, check_robot_alive, mpc_tracer_process, auto_tracking_stats, test_timer):
        if not rospy.get_param(f'/{self.ros_namespace}/test_cmd_vel', False):
            rospy.loginfo("cmd_vel 测试未启用")
            return
        round = rospy.get_param(f'/{self.ros_namespace}/round', 10)
        rospy.loginfo(f"Test Robot Walk Round: {round}")
        path_types = rospy.get_param(f'/{self.ros_namespace}/path_types', ['circle', 'square', 'scurve'])
        rospy.set_param(f'/mpc_path_tracer_node/motion_interface', '/cmd_vel')
        while round > 0:
            for path_type in path_types:
                if path_type not in self.available_path_types:
                    rospy.logerr(f"路径类型 {path_type} 不存在")
                    continue
                self.execute_path(path_type, check_robot_alive, auto_tracking_stats)
            round -= 1
            rospy.loginfo(f"剩余轮数: {round}")

    @pytest.mark.walk
    def test_robot_walk_cmd_pose(self, check_robot_ready, check_robot_alive, mpc_tracer_process, auto_tracking_stats, test_timer):
        if not rospy.get_param(f'/{self.ros_namespace}/test_cmd_pose', False):
            rospy.loginfo("cmd_pose 测试未启用")
            return
        round = rospy.get_param(f'/{self.ros_namespace}/round', 10)
        rospy.loginfo(f"Test Robot Walk Round: {round}")
        path_types = rospy.get_param(f'/{self.ros_namespace}/path_types', ['circle', 'square', 'scurve'])
        rospy.set_param(f'/mpc_path_tracer_node/motion_interface', '/cmd_pose')
        while round > 0:
            for path_type in path_types:
                if path_type not in self.available_path_types:
                    rospy.logerr(f"路径类型 {path_type} 不存在")
                    continue
                self.execute_path(path_type, check_robot_alive, auto_tracking_stats)

            round -= 1
            rospy.loginfo(f"剩余轮数: {round}")

    @pytest.mark.walk
    def test_robot_walk_cmd_pose_world(self, check_robot_ready, check_robot_alive, mpc_tracer_process, auto_tracking_stats, test_timer):
        if not rospy.get_param(f'/{self.ros_namespace}/test_cmd_pose_world', False):
            rospy.loginfo("cmd_pose_world 测试未启用")
            return
        round = rospy.get_param(f'/{self.ros_namespace}/round', 10)
        rospy.loginfo(f"Test Robot Walk Round: {round}")
        path_types = rospy.get_param(f'/{self.ros_namespace}/path_types', ['circle', 'square', 'scurve'])
        rospy.set_param(f'/mpc_path_tracer_node/motion_interface', '/cmd_pose_world')
        while round > 0:
            for path_type in path_types:
                if path_type not in self.available_path_types:
                    rospy.logerr(f"路径类型 {path_type} 不存在")
                    continue
                self.execute_path(path_type, check_robot_alive, auto_tracking_stats)

            round -= 1
            rospy.loginfo(f"剩余轮数: {round}")