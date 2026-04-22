#! /usr/bin/env python
import time
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from kuavo_msgs.msg import robotHeadMotionData  # 头部电机控制
from joint_state_publisher import JointStatePublisher

class ARControlNode(object):
    def __init__(self):
        rospy.init_node('ar_control_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._head_traj_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
        self.publisher = rospy.Publisher('/robot_tag_info', AprilTagDetectionArray, queue_size=10)
        self.subscription = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
    def pub_kuavo_head_traj(self, from_joint_state, target_joint_state):
        # 头部包含两个电机，数据格式为长度为 2 的数组
        # 第一个值代表水平方向，范围 -30 ~ 30
        # 第二个值代表竖直方向，范围 -25 ~ 25
        # 按照每次 3 度进行插值处理
        head_traj_msg = robotHeadMotionData()

        step = 3 if target_joint_state[0] > from_joint_state[0] else -3
        for i in range(from_joint_state[0], target_joint_state[0], step):
            head_traj_msg.joint_data = [i, from_joint_state[1]]
            self._head_traj_pub.publish(head_traj_msg)
            time.sleep(0.1)
        step = 3 if target_joint_state[1] > from_joint_state[1] else -3
        for i in range(from_joint_state[1], target_joint_state[1], step):
            head_traj_msg.joint_data = [target_joint_state[0], i]
            self._head_traj_pub.publish(head_traj_msg)
            time.sleep(0.1)

    def tag_callback(self, msg):
        new_msg = AprilTagDetectionArray()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.header.frame_id = 'base_link'
    
        for detection in msg.detections:
            # Make sure the detection's pose has a valid frame_id
            if detection.pose.header.frame_id == '':
                detection.pose.header.frame_id = 'camera_color_optical_frame'
    
            # 转换 - 使用最新的可用变换而不是指定时间戳的变换
            try:
                # 使用最新可用变换，不指定具体时间戳
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    detection.pose.header.frame_id,
                    rospy.Time(0),  # 使用Time(0)表示最新可用的变换
                    rospy.Duration(1.0)
                )
            
                # 注意这里需要使用 detection.pose.pose.pose 来获取实际的 Pose 对象
                transformed_pose = tf2_geometry_msgs.do_transform_pose(detection.pose.pose, transform)
        
                # 转换后的目标进行赋值
                detection.pose.pose.pose.position = transformed_pose.pose.position  # position赋值
                detection.pose.pose.pose.orientation = transformed_pose.pose.orientation  # orientation 赋值
                detection.pose.header.frame_id = 'base_link' # 重新该标签是基于base_link的坐标
    
                # 添加至队尾
                new_msg.detections.append(detection)
    
                # Broadcast transform to tf
                self.broadcast_transform(transformed_pose, detection.id)
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("TF Error: {} - Cannot transform from {} to base_link".format(e, detection.pose.header.frame_id))
                continue
    
        # 发布消息
        self.publisher.publish(new_msg)
    

    def broadcast_transform(self, pose, tag_id):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'tag_origin_' + str(tag_id)
        transform_stamped.transform.translation.x = pose.pose.position.x
        transform_stamped.transform.translation.y = pose.pose.position.y
        transform_stamped.transform.translation.z = pose.pose.position.z
        transform_stamped.transform.rotation = pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform_stamped)

def main():
    try:
        ar_control_node = ARControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
