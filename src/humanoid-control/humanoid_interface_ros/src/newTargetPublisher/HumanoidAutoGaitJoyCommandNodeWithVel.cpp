/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <string>

#include <ros/init.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "humanoid_interface_ros/newTargetPublisher/LowPassFilter.h"
#include "humanoid_interface_ros/newTargetPublisher/LowPassFilter5thOrder.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/Bool.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include "kuavo_msgs/gaitTimeName.h"
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <kuavo_common/common/json.hpp>
#include <map>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include "kuavo_msgs/robotHeadMotionData.h"

namespace ocs2
{
  using namespace humanoid;
  std::map<std::string, int> joyButtonMap = {
      {"BUTTON_STANCE", 0},
      {"BUTTON_TROT", 1},
      {"BUTTON_JUMP", 2},
      {"BUTTON_WALK", 3},
      {"BUTTON_LB", 4},
      {"BUTTON_RB", 5},
      {"BUTTON_BACK", 6},
      {"BUTTON_START", 7}
  };

  std::map<std::string, int> joyAxisMap = {
      {"AXIS_LEFT_STICK_Y", 0},
      {"AXIS_LEFT_STICK_X", 1},
      {"AXIS_LEFT_LT", 2},
      {"AXIS_RIGHT_STICK_YAW", 3},
      {"AXIS_RIGHT_STICK_Z", 4},
      {"AXIS_RIGHT_RT", 5},
      {"AXIS_LEFT_RIGHT_TRIGGER", 6},
      {"AXIS_FORWARD_BACK_TRIGGER", 7}
  };


  struct gaitTimeName_t
  {
    std::string name;
    double startTime;
  };
#define DEAD_ZONE 0.05
#define TARGET_REACHED_THRESHOLD 0.1
#define TARGET_REACHED_THRESHOLD_YAW 0.1
#define TARGET_REACHED_FEET_THRESHOLD 0.08
  class JoyControl
  {
  public:
    JoyControl(ros::NodeHandle &nodeHandle, const std::string &robotName, bool verbose = false)
        : nodeHandle_(nodeHandle),
          targetPoseCommand_(nodeHandle, robotName)
    {
      if (nodeHandle.hasParam("channel_map_path"))
      {
        std::string channel_map_path;
        nodeHandle.getParam("channel_map_path", channel_map_path);
        ROS_INFO_STREAM("Loading joystick mapping from " << channel_map_path);
        loadJoyJsonConfig(channel_map_path);
      }
      else
      {
        ROS_WARN_STREAM("No channel_map_path parameter found, using default joystick mapping.");
      }
      if (nodeHandle.hasParam("joystick_sensitivity"))
      {
        nodeHandle.getParam("joystick_sensitivity", joystickSensitivity);
        ROS_INFO_STREAM("Loading joystick sensitivity: " << joystickSensitivity);
      }
      else
      {
        ROS_WARN_STREAM("No input sensitivity parameter found, using default joystick sensitivity.");
      }
      Eigen::Vector4d joystickFilterCutoffFreq_(joystickSensitivity, joystickSensitivity, 
                                                  joystickSensitivity, joystickSensitivity);
      joystickFilter_.setParams(0.01,joystickFilterCutoffFreq_);
      old_joy_msg_.axes = std::vector<float>(8, 0.0);     // 假设有 8 个轴，默认值为 0.0
      old_joy_msg_.buttons = std::vector<int32_t>(12, 0);
      // Get node parameters
      std::string referenceFile;
      nodeHandle.getParam("/referenceFile", referenceFile);

      // loadData::loadCppDataType(referenceFile, "comHeight", com_height_);
      RobotVersion robot_version(3, 4);
      if (nodeHandle.hasParam("/robot_version"))
      {
        int robot_version_int;
        nodeHandle.getParam("/robot_version", robot_version_int);
        int major = robot_version_int / 10;
        int minor = robot_version_int % 10;
        robot_version = RobotVersion(major, minor);
      }
      auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
      default_joint_state_ = drake_interface_->getDefaultJointState();
      com_height_ = drake_interface_->getIntialHeight();
      loadData::loadCppDataType(referenceFile, "targetRotationVelocity", target_rotation_velocity_);
      loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", target_displacement_velocity_);
      loadData::loadCppDataType(referenceFile, "cmdvelLinearXLimit", c_relative_base_limit_[0]);
      loadData::loadCppDataType(referenceFile, "cmdvelAngularYAWLimit", c_relative_base_limit_[3]);


      // gait
      std::string gaitCommandFile;
      nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);
      ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
      std::vector<std::string> gaitList;
      loadData::loadStdVector(gaitCommandFile, "list", gaitList, verbose);
      gait_map_.clear();
      for (const auto &gaitName : gaitList)
      {
        gait_map_.insert({gaitName, humanoid::loadModeSequenceTemplate(gaitCommandFile, gaitName, verbose)});
      }

      mode_sequence_template_publisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 10, true);
      mode_scale_publisher_ = nodeHandle.advertise<std_msgs::Float32>(robotName + "_mpc_mode_scale", 10, true);
      cmd_vel_publisher_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
      joy_sub_ = nodeHandle_.subscribe("/joy", 10, &JoyControl::joyCallback, this);
      feet_sub_ = nodeHandle_.subscribe("/humanoid_controller/swing_leg/pos_measured", 2, &JoyControl::feetCallback, this);
      observation_sub_ = nodeHandle_.subscribe(robotName + "_mpc_observation", 10, &JoyControl::observationCallback, this);
      gait_scheduler_sub_ = nodeHandle_.subscribe<kuavo_msgs::gaitTimeName>(robotName + "_mpc_gait_time_name", 10, [this](const kuavo_msgs::gaitTimeName::ConstPtr &msg)
                                                                            {
                                                                              last_gait_rec_ = current_gait_rec_;
                                                                              current_gait_rec_.name = msg->gait_name;
                                                                              current_gait_rec_.startTime = msg->start_time; });
      policy_sub_ = nodeHandle_.subscribe<ocs2_msgs::mpc_flattened_controller>(
          robotName + "_mpc_policy",                            // topic name
          1,                                                    // queue length
          boost::bind(&JoyControl::mpcPolicyCallback, this, _1) // callback
      );

      stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
      re_start_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/re_start_robot", 10);
      head_motion_pub_ = nodeHandle_.advertise<kuavo_msgs::robotHeadMotionData>("/robot_head_motion_data", 10);
    }
    void loadJoyJsonConfig(const std::string &config_file)
    {
      nlohmann::json data_;
      std::ifstream ifs(config_file);
      ifs >> data_;
      for (auto &item : data_["JoyButton"].items())
      {
        std::cout << "button:" << item.key() << " item.value():" << item.value() << std::endl;
        if (joyButtonMap.find(item.key())!= joyButtonMap.end())
        {
          joyButtonMap[item.key()] = item.value();
        }
        else
          joyButtonMap.insert({item.key(), item.value()});
      }
      for (auto &item : data_["JoyAxis"].items())
      {
        if (joyAxisMap.find(item.key())!= joyAxisMap.end())
        {
          joyAxisMap[item.key()] = item.value();
        }
        else
          joyAxisMap.insert({item.key(), item.value()});
      }
    
    }
    void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr &msg)
    {
      const auto targetTrajSize = msg->planTargetTrajectories.stateTrajectory.size();
      auto planTargetTrajectory = msg->planTargetTrajectories.stateTrajectory[targetTrajSize - 1].value;
      std::vector<double> planTargetTrajectoryDouble(planTargetTrajectory.begin(), planTargetTrajectory.end());
      auto last_target = Eigen::Map<const vector_t>(planTargetTrajectoryDouble.data(), planTargetTrajectoryDouble.size());
      current_target_ = last_target.segment<6>(6);
    }

    void run()
    {
      ros::Rate rate(100);
      while (ros::ok())
      {
        ros::spinOnce();
        rate.sleep();
        if (!get_observation_)
        {
          // ROS_INFO_STREAM("Waiting for observation message...");
          continue;
        }
        checkAndPublishCommandLine(joystick_origin_axis_);
      }
      return;
    }
    bool checkTargetReached()
    {
      const vector_t currentPose = observation_.state.segment<6>(6);
      double xy_error = (currentPose.head(2) - current_target_.head(2)).norm();
      double yaw_error = std::abs(currentPose(3) - current_target_(3));
      return xy_error < TARGET_REACHED_THRESHOLD && yaw_error < TARGET_REACHED_THRESHOLD_YAW;
    }

    bool checkFeetContactPos()
    {
      if (current_desired_gait_ == "walk")
      {
        vector3_t lf_pos_w = vector3_t::Zero();
        vector3_t rf_pos_w = vector3_t::Zero();

        for (int i = 0; i < 4; i++)
        {
          lf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3) / 4;
          rf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3 + 12) / 4;
        }

        Eigen::Matrix<scalar_t, 3, 1> zyx;
        zyx << -observation_.state.segment<6>(6).tail(3)[0], 0, 0;
        vector3_t lf = getRotationMatrixFromZyxEulerAngles(zyx) * lf_pos_w;
        vector3_t rf = getRotationMatrixFromZyxEulerAngles(zyx) * rf_pos_w;
        vector3_t current_target = getRotationMatrixFromZyxEulerAngles(zyx) * current_target_.head(3);
        if (observation_.mode == ModeNumber::SS && std::abs(lf(0) - rf(0)) < TARGET_REACHED_FEET_THRESHOLD)
          return true;
        return false;
      }
      return true;
    }

    void checkAndPublishCommandLine(const vector_t &joystick_origin_axis)
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      geometry_msgs::Twist cmdVel_;
      cmdVel_.linear.x = 0;
      cmdVel_.linear.y = 0;
      cmdVel_.linear.z = 0;
      cmdVel_.angular.x = 0;
      cmdVel_.angular.y = 0;
      cmdVel_.angular.z = 0;
      static bool send_zero_twist = false;

      auto updated = commandLineToTargetTrajectories(joystick_origin_axis, observation_, cmdVel_);
      if (!std::any_of(updated.begin(), updated.end(), [](bool x)
                       { return x; })) // no command line detected
      {
        if (!send_zero_twist)
        {
          std::cout << "[JoyControl] send zero twist" << std::endl;
          cmd_vel_publisher_.publish(cmdVel_);
          send_zero_twist = true;
        }
        return;
      }
      send_zero_twist = false;
      cmd_vel_publisher_.publish(cmdVel_);
    }

  private:
    void feetCallback(const std_msgs::Float64MultiArray::ConstPtr &feet_msg)
    {
      feet_pos_measured_ = Eigen::Map<const Eigen::VectorXd>(feet_msg->data.data(), feet_msg->data.size());
    }
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      vector_t joystickOriginAxisFilter_ = vector_t::Zero(6);
      vector_t joystickOriginAxisTemp_ = vector_t::Zero(6);
      double alpha_ = joystickSensitivity / 1000;

      if (std::any_of(joy_msg->buttons.begin(), joy_msg->buttons.end(), [](float button) {
              return std::abs(button) > 1;
          }))
      {
        std::cout << "invalide joy msg"<<std::endl;
        return;
      }
      if (joy_msg->buttons.size()<=joyButtonMap["BUTTON_START"])
      {
        std::cerr << "[JoyController]:joy_msg has a error length, check your joystick_type!" << std::endl;
        return;
      }
      if(joy_msg->axes[joyAxisMap["AXIS_RIGHT_RT"]] < -0.5)
      {
        // 组合键
        double head_yaw = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
        double head_pitch = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]];
        head_yaw = 80.0 * head_yaw;     // +- 60deg
        head_pitch = -25.0 * head_pitch;// +- 25deg 
        // std::cout << "head_yaw: " << head_yaw << " head_pitch: " << head_pitch << std::endl;
        controlHead(head_yaw, head_pitch);
        // return;
        joystickOriginAxisTemp_.head(4) << joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]], joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]], joystick_origin_axis_[2], joystick_origin_axis_[3];
      }
      else
      {
        joystickOriginAxisTemp_.head(4) << joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]], joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
      }

      joystickOriginAxisFilter_ = joystickOriginAxisTemp_;
      // for(int i=0;i<4;i++)
      // {
      //   joystickOriginAxisFilter_[i] = alpha_ * joystickOriginAxisTemp_[i] + (1 - alpha_) * joystick_origin_axis_[i];
      // }
      joystickOriginAxisFilter_.head(4) = joystickFilter_.update(joystickOriginAxisTemp_.head(4));
      for (size_t i = 0; i < 4; i++)
      {
        joystickOriginAxisFilter_(i) = std::max(-1.0, std::min(1.0, joystickOriginAxisFilter_(i)));
      }
      joystick_origin_axis_ = joystickOriginAxisFilter_;
      // joystick_origin_axis_.head(4) << joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]], joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];

      vector_t button_trigger_axis = vector_t::Zero(6); 
      if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_START"]] && joy_msg->buttons[joyButtonMap["BUTTON_START"]])
      {
        callRealInitializeSrv();
      }

      if (joy_msg->buttons[joyButtonMap["BUTTON_LB"]])// 按下左侧侧键，切换模式
      {
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        pubModeGaitScale(0.9);
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        pubModeGaitScale(1.1);
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
        {
          current_arm_mode_ = (current_arm_mode_ > 0)? 0 : 1;
          callArmControlService(current_arm_mode_);
        }
      }
      else if (joy_msg->buttons[joyButtonMap["BUTTON_RB"]])// 按下右侧侧键，切换模式
      {
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        {
          c_relative_base_limit_[0] -= (c_relative_base_limit_[0] > 0.1) ? 0.05 : 0.0;
          c_relative_base_limit_[3] -= (c_relative_base_limit_[3] > 0.1) ? 0.05 : 0.0;
          std::cout << "cmdvelLinearXLimit: " << c_relative_base_limit_[0] << "\n"
                    << "cmdvelAngularYAWLimit: " << c_relative_base_limit_[3] << std::endl;
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        {
          c_relative_base_limit_[0] += 0.05;
          c_relative_base_limit_[3] += 0.05;
          std::cout << "cmdvelLinearXLimit: " << c_relative_base_limit_[0] << "\n"
                    << "cmdvelAngularYAWLimit: " << c_relative_base_limit_[3] << std::endl;
        }
      }
      else
        checkGaitSwitchCommand(joy_msg);


      if (joy_msg->buttons[joyButtonMap["BUTTON_BACK"]])
        callTerminateSrv();
      else if (joy_msg->axes[joyAxisMap["AXIS_FORWARD_BACK_TRIGGER"]])
      {
        button_trigger_axis[0] = joy_msg->axes[joyAxisMap["AXIS_FORWARD_BACK_TRIGGER"]];
        checkAndPublishCommandLine(button_trigger_axis);
        joystick_origin_axis_ = button_trigger_axis;
      }
      else if (joy_msg->axes[joyAxisMap["AXIS_LEFT_RIGHT_TRIGGER"]])
      {
        button_trigger_axis[1] = joy_msg->axes[joyAxisMap["AXIS_LEFT_RIGHT_TRIGGER"]];
        checkAndPublishCommandLine(button_trigger_axis);
        joystick_origin_axis_ = button_trigger_axis;
      }
      old_joy_msg_ = *joy_msg;
    }

    void checkGaitSwitchCommand(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      // 检查是否有gait切换指令
      if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
      {
        publishGaitTemplate("stance");
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
      {
        publishGaitTemplate("trot");
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_JUMP"]] && joy_msg->buttons[joyButtonMap["BUTTON_JUMP"]])
      {
        // publishGaitTemplate("jump");
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
      {
        publishGaitTemplate("walk");
      }
      else
      {
        return;
      }

      std::cout << "joycmd switch to: " << current_desired_gait_ << std::endl;
      std::cout << "turn " << (current_desired_gait_ == "stance" ? "on " : "off ") << " auto stance mode" << std::endl;
      auto_stance_mode_ = (current_desired_gait_ == "stance");
    }

    void publishGaitTemplate(const std::string &gaitName)
    {
      // 发布对应的gait模板
      humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
      mode_sequence_template_publisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      current_desired_gait_ = gaitName;
    }

    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &observation_msg)
    {
      observation_ = ros_msg_conversions::readObservationMsg(*observation_msg);
      get_observation_ = true;
    }

    scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
    {
      const scalar_t &dx = desiredBaseDisplacement(0);
      const scalar_t &dy = desiredBaseDisplacement(1);
      const scalar_t &dyaw = desiredBaseDisplacement(3);
      const scalar_t rotationTime = std::abs(dyaw) / target_rotation_velocity_;
      const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
      const scalar_t displacementTime = displacement / target_displacement_velocity_;
      return std::max(rotationTime, displacementTime);
    }

    /**
     * Converts command line to TargetTrajectories.
     * @param [in] commad_line_target_ : [deltaX, deltaY, deltaZ, deltaYaw]
     * @param [in] observation : the current observation
     */
    std::vector<bool> commandLineToTargetTrajectories(const vector_t &joystick_origin_axis, const SystemObservation &observation, geometry_msgs::Twist &cmdVel)
    {
      std::vector<bool> updated(6, false);
      Eigen::VectorXd limit_vector(4);
      limit_vector << c_relative_base_limit_[0], c_relative_base_limit_[1], c_relative_base_limit_[2], c_relative_base_limit_[3];
      if (joystick_origin_axis.cwiseAbs().maxCoeff() < DEAD_ZONE)
        return updated; // command line is zero, do nothing

      commad_line_target_.head(4) = joystick_origin_axis.head(4).cwiseProduct(limit_vector);

      const vector_t currentPose = observation.state.segment<6>(6);
      // vector_t target(6);
      if (joystick_origin_axis.head(2).cwiseAbs().maxCoeff() > DEAD_ZONE)
      { // base p_x, p_y are relative to current state
        // double dx = commad_line_target_(0) * cos(currentPose(3)) - commad_line_target_(1) * sin(currentPose(3));
        // double dy = commad_line_target_(0) * sin(currentPose(3)) + commad_line_target_(1) * cos(currentPose(3));
        // current_target_(0) = currentPose(0) + dx;
        // current_target_(1) = currentPose(1) + dy;
        cmdVel.linear.x = commad_line_target_(0);
        cmdVel.linear.y = commad_line_target_(1);
        updated[0] = true;
        updated[1] = true;
        // std::cout << "base displacement: " << dx << ", " << dy << std::endl;
      }
      // base z relative to the default height
      if (std::abs(joystick_origin_axis(2)) > DEAD_ZONE)
      {
        updated[2] = true;
        // current_target_(2) = com_height_ + commad_line_target_(2);
        cmdVel.linear.z = commad_line_target_(2);
        std::cout << "base height: " << current_target_(2) << std::endl;
      }
      else
      {
        // current_target_(2) = com_height_;
        cmdVel.linear.z = 0.0;
      }

      // theta_z relative to current
      if (std::abs(joystick_origin_axis(3)) > DEAD_ZONE)
      {
        updated[3] = true;
        // current_target_(3) = currentPose(3) + commad_line_target_(3) * M_PI / 180.0;
        cmdVel.angular.z = commad_line_target_(3);
      }

      return updated;
    }

    inline void pubModeGaitScale(float scale)
    {
      total_mode_scale_ *= scale;
      ROS_INFO_STREAM("[JoyControl] Publish scale: " << scale << ", Total mode scale: " << total_mode_scale_);
      std_msgs::Float32 msg;
      msg.data = scale;
      mode_scale_publisher_.publish(msg);
    }
    bool callArmControlService(int mode)
    {
      ros::ServiceClient client = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("humanoid_change_arm_ctrl_mode");
      kuavo_msgs::changeArmCtrlMode srv;
      srv.request.control_mode = mode; 

      if (client.call(srv))
      {
        ROS_INFO("changeArmCtrlMode call succeeded, received response: %s", srv.response.result ? "Success" : "Failure");
        return srv.response.result; 
      }
      else
      {
        ROS_ERROR("Failed to call service change_arm_ctrl_mode");
        return false;
      }
    }
    void callRealInitializeSrv()
    {
      ros::ServiceClient client = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/real_initial_start");
      std_srvs::Trigger srv;

      // 调用服务
      if (client.call(srv))
      {
        ROS_INFO("[JoyControl] Service call successful");
      }
      else
      {
        ROS_ERROR("Failed to callRealInitializeSrv service, use publish topic.");
        std_msgs::Bool msg;
        msg.data = true;
        re_start_pub_.publish(msg);
      }
    }
    void callTerminateSrv()
    {
      std::cout << "tigger callTerminateSrv" << std::endl;
      for (int i = 0; i < 5; i++)
      {
        std_msgs::Bool msg;
        msg.data = true;
        stop_pub_.publish(msg);
        ::ros::Duration(0.1).sleep();
      }
    }

    void controlHead(double head_yaw, double head_pitch)
    {
      kuavo_msgs::robotHeadMotionData msg;
      msg.joint_data.resize(2);
      msg.joint_data[0] = head_yaw;
      msg.joint_data[1] = head_pitch;
      head_motion_pub_.publish(msg);
    }

  private:
    ros::NodeHandle nodeHandle_;
    TargetTrajectoriesRosPublisher targetPoseCommand_;
    ros::Subscriber joy_sub_;
    ros::Subscriber feet_sub_;
    ros::Subscriber observation_sub_;
    ros::Subscriber gait_scheduler_sub_;
    ros::Subscriber policy_sub_;
    bool get_observation_ = false;
    vector_t current_target_ = vector_t::Zero(6);
    std::string current_desired_gait_ = "stance";
    gaitTimeName_t current_gait_rec_{"stance", 0.0}, last_gait_rec_{"stance", 0.0};
    bool auto_stance_mode_ = true;
    bool reached_target_ = false;
    scalar_t target_displacement_velocity_;
    scalar_t target_rotation_velocity_;
    scalar_t com_height_;
    vector_t default_joint_state_ = vector_t::Zero(12);
    vector_t commad_line_target_ = vector_t::Zero(6);
    vector_t joystick_origin_axis_ = vector_t::Zero(6);
    sensor_msgs::Joy old_joy_msg_;
    int current_arm_mode_{1};
    double joystickSensitivity = 100;
    LowPassFilter5thOrder joystickFilter_;

    ocs2::scalar_array_t c_relative_base_limit_{0.4, 0.2, 0.3, 0.4};
    ocs2::SystemObservation observation_;
    ros::Publisher mode_sequence_template_publisher_;
    ros::Publisher mode_scale_publisher_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher stop_pub_;
    ros::Publisher re_start_pub_;
    ros::Publisher head_motion_pub_;
    float total_mode_scale_{1.0};
    bool button_start_released_{true};
    // TargetTrajectories current_target_traj_;
    std::mutex target_mutex_;
    vector_t feet_pos_measured_ = vector_t::Zero(24);

    std::map<std::string, humanoid::ModeSequenceTemplate> gait_map_;
  };
}

int main(int argc, char *argv[])
{
  const std::string robotName = "humanoid";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_joy_command_node");
  ::ros::NodeHandle nodeHandle;

  ocs2::JoyControl joyControl(nodeHandle, robotName);
  joyControl.run();

  return 0;
}
