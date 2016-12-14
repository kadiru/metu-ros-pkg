/*
 * human_controller.cpp
 * Copyright (c) 2012, Kadir Firat Uyanik, KOVAN Research Lab, METU
 * kadir@ceng.metu.edu.tr
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of KOVAN Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "al_human/human_controller.h"
#include <pluginlib/class_list_macros.h>

namespace human_controller_ns
{
  /// Controller initialization in non-realtime
  bool
  HumanControllerClass::init (pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    std::string joint_name;
    if (!n.getParam ("joint_name", joint_name))
    {
      ROS_ERROR ("No joint given in namespace: '%s')", n.getNamespace ().c_str ());
      return false;
    }

    joint_state_ = robot->getJointState (joint_name);
    if (!joint_state_)
    {
      ROS_ERROR ("HumanController could not find joint named '%s'", joint_name.c_str ());
      return false;
    }

    srv_ = n.advertiseService ("set_joints", &HumanControllerClass::srvSetJoints, this);

    return true;
  }

  /// Controller startup in realtime
  void
  HumanControllerClass::starting ()
  {
    init_pos_ = joint_state_->position_;
  }

  /// Controller update loop in realtime
  void
  HumanControllerClass::update ()
  {
    double desired_pos = init_pos_ + 15 * sin (ros::Time::now ().toSec ());
    double current_pos = joint_state_->position_;
    joint_state_->commanded_effort_ = -10 * (current_pos - desired_pos);
  }

  /// Controller stopping in realtime
  void
  HumanControllerClass::stopping ()
  {
  }

  bool
  HumanControllerClass::srvSetJoints (al_human::SetJoints::Request& req, al_human::SetJoints::Response& res)
  {
    std::cout << "service called" << std::endl;

    return true;
  }

} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(al_human, HumanControllerPlugin,
    human_controller_ns::HumanControllerClass,
    pr2_controller_interface::Controller)
