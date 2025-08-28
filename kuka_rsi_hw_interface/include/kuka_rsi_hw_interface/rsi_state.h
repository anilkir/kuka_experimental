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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 * Modified by: Anil Kircaliali <anilkircaliali@gmail.com>
*/

#ifndef KUKA_RSI_HW_INTERFACE_RSI_STATE_
#define KUKA_RSI_HW_INTERFACE_RSI_STATE_

#include <string>
#include <tinyxml.h>
#include <kuka_resources/kuka_common.h>

namespace kuka_rsi_hw_interface
{
  class RSIState
  {
  private:
    std::string xml_doc_;

  public:
    RSIState(int n_dof = 6) :
      positions(n_dof, 0.0),
      initial_positions(n_dof, 0.0),
      cart_position(n_dof, 0.0),
      initial_cart_position(n_dof, 0.0)
    {
      xml_doc_.resize(1024);
    }

    RSIState(std::string xml_doc, int n_dof, kuka_rsi_common::RSIConfigType config_type = kuka_rsi_common::RSIConfigType::SINGLE_MOTOR_EXTRUDER);

    // AIPOS
    std::vector<double> positions;
    // ASPos
    std::vector<double> initial_positions;
    // RIst
    std::vector<double> cart_position;
    // RSol
    std::vector<double> initial_cart_position;
    // IPOC
    unsigned long long ipoc;

    // Feedback commands for extruders:
    unsigned long long current_cmd_id;        // CurCmdID
    double current_robot_speed;   // RobotSpeed
    double current_motor_speed;   // MotorSpeed
    // Feedback commands for FiberGun
    // MainServoSpeed
    double current_main_servo_speed;
    // BladeCount
    unsigned long long current_blade_count;
    // ResinSprayState
    unsigned long long current_resin_spray_state;
    // ChuteAirState
    unsigned long long current_chute_air_state;

  };

RSIState::RSIState(std::string xml_doc, int n_dof, kuka_rsi_common::RSIConfigType config_type) :
  xml_doc_(xml_doc),
  positions(n_dof, 0.0),
  initial_positions(n_dof, 0.0),
  cart_position(n_dof, 0.0),
  initial_cart_position(n_dof, 0.0)
{
  switch (config_type) {
    case kuka_rsi_common::RSIConfigType::SINGLE_MOTOR_EXTRUDER:
      current_cmd_id = 0;
      current_robot_speed = 0;
      current_motor_speed = 0;
      break;
    // TODO: Fix the DUAL MOTOR EXTRUDER definition here, it needs to have two motors
    case kuka_rsi_common::RSIConfigType::DUAL_MOTOR_EXTRUDER:
      current_cmd_id = 0;
      current_robot_speed = 0;
      // TODO: Add more motors
      current_motor_speed = 0;
      break;
    case kuka_rsi_common::RSIConfigType::FIBERGUN:
      current_main_servo_speed = 0;
      current_blade_count = 0;
      current_resin_spray_state = 0;
      current_chute_air_state = 0;
      break;
  }

  // Parse message from robot
  TiXmlDocument bufferdoc;
  bufferdoc.Parse(xml_doc_.c_str());
  // Get the Rob node:
  TiXmlElement* rob = bufferdoc.FirstChildElement("Rob");
  // Extract axis specific actual position
  TiXmlElement* AIPos_el = rob->FirstChildElement("AIPos");
  AIPos_el->Attribute("A1", &positions[0]);
  AIPos_el->Attribute("A2", &positions[1]);
  AIPos_el->Attribute("A3", &positions[2]);
  AIPos_el->Attribute("A4", &positions[3]);
  AIPos_el->Attribute("A5", &positions[4]);
  AIPos_el->Attribute("A6", &positions[5]);
  // Extract axis specific setpoint position
  TiXmlElement* ASPos_el = rob->FirstChildElement("ASPos");
  ASPos_el->Attribute("A1", &initial_positions[0]);
  ASPos_el->Attribute("A2", &initial_positions[1]);
  ASPos_el->Attribute("A3", &initial_positions[2]);
  ASPos_el->Attribute("A4", &initial_positions[3]);
  ASPos_el->Attribute("A5", &initial_positions[4]);
  ASPos_el->Attribute("A6", &initial_positions[5]);
  // If an external track is defined, extract the external axis positions
  if (n_dof == 7) {
    // Extract external axis specific actual position
    TiXmlElement* EIPos_el = rob->FirstChildElement("EIPos");
    EIPos_el->Attribute("E1", &positions[6]);
    // Extract external axis specific setpoint position
    TiXmlElement* ESPos_el = rob->FirstChildElement("ESPos");
    ESPos_el->Attribute("E1", &initial_positions[6]);
  }
   
  // Extract cartesian actual position
  TiXmlElement* RIst_el = rob->FirstChildElement("RIst");
  RIst_el->Attribute("X", &cart_position[0]);
  RIst_el->Attribute("Y", &cart_position[1]);
  RIst_el->Attribute("Z", &cart_position[2]);
  RIst_el->Attribute("A", &cart_position[3]);
  RIst_el->Attribute("B", &cart_position[4]);
  RIst_el->Attribute("C", &cart_position[5]);
  // Extract cartesian setpoint position
  TiXmlElement* RSol_el = rob->FirstChildElement("RSol");
  RSol_el->Attribute("X", &initial_cart_position[0]);
  RSol_el->Attribute("Y", &initial_cart_position[1]);
  RSol_el->Attribute("Z", &initial_cart_position[2]);
  RSol_el->Attribute("A", &initial_cart_position[3]);
  RSol_el->Attribute("B", &initial_cart_position[4]);
  RSol_el->Attribute("C", &initial_cart_position[5]);

  // TODO: Add DUAL_MOTOR_EXTRUDER as well
  if (config_type == kuka_rsi_common::RSIConfigType::SINGLE_MOTOR_EXTRUDER) {
    TiXmlElement* current_cmd_id_el = rob->FirstChildElement("CurCmdID");
    current_cmd_id = std::stoull(current_cmd_id_el->FirstChild()->Value());
    // Get the current robot speed (used in Type 2b tasks with variable global speed)
    TiXmlElement* current_robot_speed_el = rob->FirstChildElement("RobotSpeed");
    current_robot_speed = std::stod(current_robot_speed_el->FirstChild()->Value());
    ROS_INFO("RSI Current Robot Speed: %f", current_robot_speed);
    // Get the current motor speed (used in Type 2b tasks and ignored in Type 3 and 4 tasks)
    TiXmlElement* current_motor_speed_el = rob->FirstChildElement("MotorSpeed");
    ROS_INFO("RSI Current Motor Speed: %f", current_motor_speed);
    current_motor_speed = std::stod(current_motor_speed_el->FirstChild()->Value());
  } else if (config_type == kuka_rsi_common::RSIConfigType::FIBERGUN) {
    TiXmlElement* current_main_servo_speed_el = rob->FirstChildElement("MainServoSpeed");
    current_main_servo_speed = std::stod(current_main_servo_speed_el->FirstChild()->Value());
    TiXmlElement* current_blade_count_el = rob->FirstChildElement("BladeCount");
    current_blade_count = std::stoull(current_blade_count_el->FirstChild()->Value());
    TiXmlElement* current_resin_spray_state_el = rob->FirstChildElement("ResinSprayState");
    current_resin_spray_state = std::stoull(current_resin_spray_state_el->FirstChild()->Value());
    TiXmlElement* current_chute_air_state_el = rob->FirstChildElement("ChuteAirState");
    current_chute_air_state = std::stoull(current_chute_air_state_el->FirstChild()->Value());
  }

  // Get the IPOC timestamp
  TiXmlElement* ipoc_el = rob->FirstChildElement("IPOC");
  ipoc = std::stoull(ipoc_el->FirstChild()->Value());
}

} // namespace kuka_rsi_hw_interface

#endif
