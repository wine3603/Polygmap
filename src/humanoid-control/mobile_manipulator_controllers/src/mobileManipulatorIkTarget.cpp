#include <iostream>
#include <ros/ros.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "kuavo_msgs/twoArmHandPoseCmd.h"
#include "ros/service_server.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <kuavo_msgs/setMmCtrlFrame.h>

namespace mobile_manipulator_controller
{
using namespace ocs2;

enum class FrameType
{
  CurrentFrame = 0,    // keep current frame
  WorldFrame = 1,     
  LocalFrame = 2,
  VRFrame = 3,
  MmWorldFrame = 4
};

struct IkCmd
{
    Eigen::Vector3d pos;     // hand pos
    Eigen::Quaterniond quat; // hand quaternion
};

struct BasePoseCmd
{
  int dim;
  Eigen::Matrix<double, 6, 1> pose;
};

class MobileManipulatorIkTarget {
  public:
    MobileManipulatorIkTarget(ros::NodeHandle& nodeHandle, const std::string& robotName)
    : nodeHandle_(nodeHandle)
    {
      targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle_, robotName));
      // Get use_quest3_utils parameter

      if (!nodeHandle_.getParam("use_quest3_utils", use_quest3_utils_)) {
        ROS_WARN("Parameter 'use_quest3_utils' not found, using default value: false");
      }
      while (!nodeHandle_.getParam("com_height", comHeight_)) {
        ROS_WARN("Waiting for parameter 'com_height'...");
        ros::Duration(0.5).sleep();  // 每0.5秒检查一次
      }
      ROS_INFO("Got com_height: %f", comHeight_);

      ROS_INFO("use_quest3_utils: %d", use_quest3_utils_); 
      if(use_quest3_utils_)
        frameType_ = FrameType::VRFrame;

      // observation subscriber
      auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        // std::cout << "latestObservation_.state.size(): " << latestObservation_.state.size() << std::endl;
        // std::cout << "latestObservation_.state: " << latestObservation_.state.transpose() << std::endl;
        if(!observationReceived_)
          observationReceived_ = true;
      };
      auto humanoidObservationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        latestHumanoidObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        if(!humanoidObservationReceived_)
          humanoidObservationReceived_ = true;
      };

      observationSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1, observationCallback);
      humanoidObservationSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_observation>("humanoid_wbc_observation", 1, humanoidObservationCallback);
      ikCmdSubscriber_ = nodeHandle_.subscribe<kuavo_msgs::twoArmHandPoseCmd>("/mm/two_arm_hand_pose_cmd", 10, &MobileManipulatorIkTarget::ikCmdCallback, this);
      basePoseCmdSubscriber_ = nodeHandle_.subscribe<std_msgs::Float64MultiArray>(robotName + "/base_pose_cmd", 10, &MobileManipulatorIkTarget::basePoseCmdCallback, this);
      
      // Add service server for setting use_quest3_utils
      setQuest3UtilsService_ = nodeHandle_.advertiseService("set_quest3_utils", &MobileManipulatorIkTarget::setQuest3UtilsCallback, this);
      setFrameTypeService_ = nodeHandle_.advertiseService("set_mm_ctrl_frame", &MobileManipulatorIkTarget::setFrameTypeCallback, this);
      getFrameTypeService_ = nodeHandle_.advertiseService("get_mm_ctrl_frame", &MobileManipulatorIkTarget::getFrameTypeCallback, this);
    }

    void run()
    {
      ros::spin();
    }

  private:
    bool setQuest3UtilsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
      use_quest3_utils_ = req.data;
      ROS_INFO("use_quest3_utils set to: %d", use_quest3_utils_);
      res.success = true;
      res.message = "Successfully set use_quest3_utils to " + std::to_string(use_quest3_utils_);
      frameType_ = FrameType::VRFrame;
      return true;
    }

    bool setFrameTypeCallback(kuavo_msgs::setMmCtrlFrame::Request& req, kuavo_msgs::setMmCtrlFrame::Response& res)
    {
      FrameType frameType = static_cast<FrameType>(req.frame);
      if(frameType == FrameType::CurrentFrame)
      {
        ROS_INFO("[setFrameTypeCallback]: Keep CurrentFrame");
      }
      else 
      {
        frameType_ = frameType;
        ROS_INFO("[setFrameTypeCallback]: Set frameType_ to %d", frameType_);
      }
      res.result = true;
      res.message = "Successfully set frameType_ to " + std::to_string(static_cast<int>(frameType_));
      res.currentFrame = static_cast<int>(frameType_);
      return true;
    }

    bool getFrameTypeCallback(kuavo_msgs::setMmCtrlFrame::Request& req, kuavo_msgs::setMmCtrlFrame::Response& res)
    {
      res.result = true;
      res.message = "Successfully get frameType_ to " + std::to_string(static_cast<int>(frameType_));
      res.currentFrame = static_cast<int>(frameType_);
      return true;
    }

    void basePoseCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      newBasePoseReceived_ = true;
      const auto& pose = msg->data;
      if(pose.size() > 6)
      {
        ROS_ERROR_STREAM("Invalid base pose command size: " << pose.size());
        return;
      }
      basePoseCmd_.dim = pose.size();
      for(int i = 0; i < pose.size(); i++)
        basePoseCmd_.pose(i) = pose[i];
    }

    void ikCmdCallback(const kuavo_msgs::twoArmHandPoseCmd::ConstPtr& msg)
    {
      // std::cout << "ikCmdCallback" << std::endl;
      // std::cout << "use_quest3_utils_: " << use_quest3_utils_ << std::endl;
      if(!observationReceived_)
      {
        ROS_WARN_STREAM("No observation received yet, cannot compute target trajectories");
        return;
      }
      auto &hand_poses = msg->hand_poses;
      // std::cout << "hand_poses.frame: " << msg->frame << std::endl;
      FrameType frameType = static_cast<FrameType>(msg->frame);
      // if(frameType == FrameType::CurrentFrame)
      //   std::cout << "CurrentFrame" << std::endl;
      // else if(frameType == FrameType::WorldFrame)
      //   std::cout << "WorldFrame" << std::endl;
      // else if(frameType == FrameType::LocalFrame)
      //   std::cout << "LocalFrame" << std::endl;

      if (frameType != FrameType::CurrentFrame)
      {
        frameType_ = frameType;
        std::cout << "frameType_ set to: " << static_cast<int>(frameType_) << std::endl;
      }
      // std::cout << "frameType_: " << static_cast<int>(frameType_) << std::endl;
      // std::cout << "comHeight_: " << comHeight_ << std::endl;


      IkCmd ik_cmd_l, ik_cmd_r;
      // left
      ik_cmd_l.pos << hand_poses.left_pose.pos_xyz[0], hand_poses.left_pose.pos_xyz[1], hand_poses.left_pose.pos_xyz[2];
      ik_cmd_l.quat = Eigen::Quaterniond(hand_poses.left_pose.quat_xyzw[3],
                              hand_poses.left_pose.quat_xyzw[0], hand_poses.left_pose.quat_xyzw[1], hand_poses.left_pose.quat_xyzw[2]);
      // right
      ik_cmd_r.pos << hand_poses.right_pose.pos_xyz[0], hand_poses.right_pose.pos_xyz[1], hand_poses.right_pose.pos_xyz[2];
      ik_cmd_r.quat = Eigen::Quaterniond(hand_poses.right_pose.quat_xyzw[3],
                              hand_poses.right_pose.quat_xyzw[0], hand_poses.right_pose.quat_xyzw[1], hand_poses.right_pose.quat_xyzw[2]);

      // p_be: position in VR frame, quat_be: orientation in VR frame, observation: humanoid observation
      auto transPoseFromVRFrameToMmWorld = [](const Eigen::Vector3d& p_be, const Eigen::Quaterniond& quat_be, const SystemObservation& observation)
      {
        Eigen::Vector3d p_mw_b = (Eigen::Vector3d() << observation.state.segment<2>(6), 0).finished();
        Eigen::Vector3d zyx = (Eigen::Vector3d() << observation.state(9), 0, 0).finished();
        Eigen::Matrix3d R_mw_b = ocs2::getRotationMatrixFromZyxEulerAngles(zyx);
        Eigen::Vector3d p_mw_e = p_mw_b + R_mw_b * p_be;
        Eigen::Matrix3d R_mw_e = R_mw_b * quat_be.toRotationMatrix();
        Eigen::Quaterniond quat_mw_e(R_mw_e);
        return std::make_pair(p_mw_e, quat_mw_e);
      };
      // p_le: position in local frame, quat_le: orientation in local frame, observation: humanoid observation
      auto transPoseFromLocalFrameToWorld = [this](const Eigen::Vector3d& p_le, const Eigen::Quaterniond& quat_le, const SystemObservation& observation)
      {
        Eigen::Vector3d p_mw_l = (Eigen::Vector3d() << observation.state.segment<2>(6), -comHeight_).finished();
        Eigen::Vector3d zyx = (Eigen::Vector3d() << observation.state(9), 0, 0).finished();
        Eigen::Matrix3d R_mw_l = ocs2::getRotationMatrixFromZyxEulerAngles(zyx);
        Eigen::Vector3d p_mw_e = p_mw_l + R_mw_l * p_le;
        Eigen::Matrix3d R_mw_e = R_mw_l * quat_le.toRotationMatrix();
        Eigen::Quaterniond quat_mw_e(R_mw_e);
        return std::make_pair(p_mw_e, quat_mw_e);
      };
      // p_we: position in world frame, quat_we: orientation in world frame, observation: humanoid observation
      auto transPoseFromWorldFrameToMmWorld = [this](const Eigen::Vector3d& p_we, const Eigen::Quaterniond& quat_we, const SystemObservation& observation)
      {
        Eigen::Vector3d p_mw_w = (Eigen::Vector3d() << 0.0, 0.0, -comHeight_).finished();
        // Eigen::Matrix3d R_mw_w;
        // R_mw_w.setIdentity();
        Eigen::Vector3d p_mw_e = p_mw_w + p_we;
        Eigen::Quaterniond quat_mw_e = quat_we;
        return std::make_pair(p_mw_e, quat_mw_e);
      };


      if (humanoidObservationReceived_)
      {
        if(frameType_ == FrameType::VRFrame)
        {
          std::tie(ik_cmd_l.pos, ik_cmd_l.quat) = transPoseFromVRFrameToMmWorld(ik_cmd_l.pos, ik_cmd_l.quat, latestHumanoidObservation_);
          std::tie(ik_cmd_r.pos, ik_cmd_r.quat) = transPoseFromVRFrameToMmWorld(ik_cmd_r.pos, ik_cmd_r.quat, latestHumanoidObservation_);
        }
        else if(frameType_ == FrameType::WorldFrame)
        {
          std::tie(ik_cmd_l.pos, ik_cmd_l.quat) = transPoseFromWorldFrameToMmWorld(ik_cmd_l.pos, ik_cmd_l.quat, latestHumanoidObservation_);
          std::tie(ik_cmd_r.pos, ik_cmd_r.quat) = transPoseFromWorldFrameToMmWorld(ik_cmd_r.pos, ik_cmd_r.quat, latestHumanoidObservation_);
        }
        else if(frameType_ == FrameType::LocalFrame)
        {
          std::tie(ik_cmd_l.pos, ik_cmd_l.quat) = transPoseFromLocalFrameToWorld(ik_cmd_l.pos, ik_cmd_l.quat, latestHumanoidObservation_);
          std::tie(ik_cmd_r.pos, ik_cmd_r.quat) = transPoseFromLocalFrameToWorld(ik_cmd_r.pos, ik_cmd_r.quat, latestHumanoidObservation_);
        }
      }

      
      auto target = goalPoseToTargetTrajectories(ik_cmd_l, ik_cmd_r, latestObservation_);
      // const auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(target);
    }

    TargetTrajectories goalPoseToTargetTrajectories(const IkCmd& cmd_l, const IkCmd& cmd_r, const SystemObservation& observation) {
      // time trajectory
      const scalar_array_t timeTrajectory{observation.time};
      // state trajectory: 3 + 4 for desired position vector and orientation quaternion
      auto l_pose = (vector_t(7) << cmd_l.pos, cmd_l.quat.coeffs()).finished(); 
      auto r_pose = (vector_t(7) << cmd_r.pos, cmd_r.quat.coeffs()).finished();

      vector_t target;
      if(newBasePoseReceived_)
      {
        target = (vector_t(14 + basePoseCmd_.dim) << basePoseCmd_.pose, l_pose, r_pose).finished();
        newBasePoseReceived_ = false;
      }
      else
      {
        target = (vector_t(14) << l_pose, r_pose).finished();        
      }
      const vector_array_t stateTrajectory{target};
      // input trajectory
      const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

      return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

  private:
    ros::NodeHandle& nodeHandle_;
    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
    ::ros::Subscriber observationSubscriber_;
    ::ros::Subscriber humanoidObservationSubscriber_;
    ::ros::Subscriber ikCmdSubscriber_;
    ::ros::Subscriber basePoseCmdSubscriber_;
    ::ros::ServiceServer setQuest3UtilsService_;
    ::ros::ServiceServer setFrameTypeService_;
    ::ros::ServiceServer getFrameTypeService_;

    SystemObservation latestObservation_;
    SystemObservation latestHumanoidObservation_;
    bool observationReceived_ = false;
    bool humanoidObservationReceived_ = false;
    bool newBasePoseReceived_ = false;
    BasePoseCmd basePoseCmd_;
    bool use_quest3_utils_ = false;
    FrameType frameType_ = FrameType::MmWorldFrame;
    double comHeight_ = 0.0;
};
}  // namespace mobile_manipulator_controller


int main(int argc, char* argv[]) {
  const std::string robotName = "mobile_manipulator";
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  mobile_manipulator_controller::MobileManipulatorIkTarget ikTarget(nodeHandle, robotName);
  ikTarget.run();
  // Successful exit
  return 0;
}
