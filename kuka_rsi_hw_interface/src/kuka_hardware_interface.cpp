/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

#include <stdexcept>


namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface(int n_dof, kuka_rsi_common::RSIConfigType config_type) :
    n_dof_(n_dof), config_type_(config_type),
    joint_position_(n_dof, 0.0), joint_velocity_(n_dof, 0.0), joint_effort_(n_dof, 0.0), joint_position_command_(n_dof, 0.0), joint_velocity_command_(n_dof, 0.0),
    joint_effort_command_(n_dof, 0.0), joint_names_(n_dof), rsi_initial_joint_positions_(n_dof, 0.0), rsi_joint_position_corrections_(n_dof, 0.0), ipoc_(0)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'controller_joint_names' on the parameter server.");
  }

  //Create ros_control interfaces
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                             &joint_effort_[i]));

    // Create joint position control interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_rsi_hardware_interface");
}

KukaHardwareInterface::~KukaHardwareInterface()
{

}

bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }

  if (rt_rsi_pub_->trylock()){
    rt_rsi_pub_->msg_.data = in_buffer_;
    rt_rsi_pub_->unlockAndPublish();
  }

  rsi_state_ = RSIState(in_buffer_, n_dof_, config_type_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    if (i < 6) {
      joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    }
    if (i == 6) {
      joint_position_[i] = rsi_state_.positions[i] / 1000; // For the 7th joint, mm to meters needed since track is in mm
    }
  }
  ipoc_ = rsi_state_.ipoc;

  if (config_type_ == kuka_rsi_common::RSIConfigType::SINGLE_MOTOR_EXTRUDER) {
    current_cmd_id_ = rsi_state_.current_cmd_id;
    current_motor_speed_ = rsi_state_.current_mot_spd;

    if (rt_current_cmd_id_pub_->trylock()) {
      rt_current_cmd_id_pub_->msg_.data = current_cmd_id_;
      rt_current_cmd_id_pub_->unlockAndPublish();
    }
    if (rt_current_mot_spd_pub_->trylock()) {
      rt_current_mot_spd_pub_->msg_.data = current_motor_speed_;
      rt_current_mot_spd_pub_->unlockAndPublish();
    }
  }
  else if (config_type_ == kuka_rsi_common::RSIConfigType::FIBERGUN)
  {
    current_main_servo_speed_ = rsi_state_.current_main_servo_speed;
    current_blade_count_ = rsi_state_.current_blade_count;
    current_resin_spray_state_ = rsi_state_.current_resin_spray_state;
    current_chute_air_state_ = rsi_state_.current_chute_air_state;

    if (rt_current_main_servo_speed_pub_->trylock()) {
      rt_current_main_servo_speed_pub_->msg_.data = current_main_servo_speed_;
      rt_current_main_servo_speed_pub_->unlockAndPublish();
    }
    if (rt_current_blade_count_pub_->trylock()) {
      rt_current_blade_count_pub_->msg_.data = current_blade_count_;
      rt_current_blade_count_pub_->unlockAndPublish();
    }
    if (rt_current_resin_spray_state_pub_->trylock()) {
      rt_current_resin_spray_state_pub_->msg_.data = current_resin_spray_state_;
      rt_current_resin_spray_state_pub_->unlockAndPublish();
    }
    if (rt_current_chute_air_state_pub_->trylock()) {
      rt_current_chute_air_state_pub_->msg_.data = current_chute_air_state_;
      rt_current_chute_air_state_pub_->unlockAndPublish();
    }
  }

  return true;
}

bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < 6; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }
  if (n_dof_ == 7) {
    // For the 7th joint, we assume it's a linear track and convert to mm
    rsi_joint_position_corrections_[6] = (joint_position_command_[6] * 1000) - rsi_initial_joint_positions_[6];
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, n_dof_).xml_doc;
  server_->send(out_buffer_);

  return true;
}

void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_, n_dof_, config_type_);
  for (std::size_t i = 0; i < 6; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
  }
  if (n_dof_ == 7) {
    // For the 7th joint, we assume it's a linear track and convert to meters
    joint_position_[6] = rsi_state_.positions[6] / 1000;
    joint_position_command_[6] = joint_position_[6];
    rsi_initial_joint_positions_[6] = rsi_state_.initial_positions[6]; // Convert initial position to meters
  }

  ROS_INFO("Joint position 6: %f, joint position command 6: %f, initial position 6: %f",
           joint_position_[6], joint_position_command_[6], rsi_initial_joint_positions_[6]);

  // ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "rsi joint position corrections: " << typeid(rsi_joint_position_corrections_).name());
  
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, n_dof_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 5 seconds
  server_->set_timeout(5000);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");

}

void KukaHardwareInterface::configure()
{
  std::string address_mode;
  nh_.param<std::string>("rsi/address_mode", address_mode, "sim"); // default to "sim"

  std::string param_addr;
  if (address_mode == "sim") {
    param_addr = "rsi/listen_address/sim";
  } else if (address_mode == "real") {
    param_addr = "rsi/listen_address/real";
  } 
  const std::string param_port = "rsi/listen_port";

  if (nh_.getParam(param_addr, local_host_) && nh_.getParam(param_port, local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    std::string msg = "Failed to get RSI listen address or listen port from"
    " parameter server (looking for '" + param_addr + "' and '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
  rt_rsi_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc", 3));
  rt_current_cmd_id_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int32>(nh_, "current_cmd_id", 3));
  rt_current_mot_spd_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh_, "current_motor_speed", 3));
  rt_current_main_servo_speed_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh_, "current_main_servo_speed", 3));
  rt_current_blade_count_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int32>(nh_, "current_blade_count", 3));
  rt_current_resin_spray_state_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int32>(nh_, "current_resin_spray_state", 3));
  rt_current_chute_air_state_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int32>(nh_, "current_chute_air_state", 3));
}

} // namespace kuka_rsi_hardware_interface
