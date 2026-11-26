/*
 * SSLPlayer.cpp
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
 * IEEE METU Robotics and Automation Society (IEEE METU RAS)
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
 *     * Neither the name of IEEE METU RAS nor the names of its
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

#include "ssl_player/SSLPlayer.h"

namespace ssl
{

  SSLPlayer::SSLPlayer (ros::NodeHandle* nh)
  {
    nh_ = nh;
    if(nh_ == NULL)
    {
      ROS_ERROR("nodehandler is NULL, exiting!");
      exit(-2);
    }

    team_ = naming::team::BLUE_TEAM;
    id_ = 0;
    wheel_radius_ = 0.025;
    robot_radius_ = 0.085;

    if (!nh_->getParam ("team", team_))
      ROS_WARN("no such parameter as <team>");
    if (!nh_->getParam ("id", id_))
      ROS_WARN("no such parameter as <id>");
    if (!nh_->getParam ("/Kinematics/WheelRadius", wheel_radius_))
      ROS_WARN("no such parameter as <Kinematics/WheelRadius>");
    if (!nh_->getParam ("/Kinematics/RobotRadius", robot_radius_))
      ROS_WARN("no such parameter as <Kinematics/RobotRadius>");

    //obtain the wheel placement information
    std::string param_name;
    uint wheel_id = 0;
    bool wheel_exist = true;
    while (wheel_exist)
    {
      param_name = "/Kinematics/WheelPlacements/Wheel";
      std::stringstream s;
      s << (int)wheel_id;
      param_name.append (s.str ());
      int wheel_placement = 0;
      wheel_exist = nh_->getParam (param_name.c_str (), wheel_placement);
      if (wheel_exist)
      {
        wheel_placements_.push_back ((float)wheel_placement);
        wheel_id++;
      }
    }
    wheel_vels_.resize (wheel_placements_.size ());
    if (!wheel_placements_.size ())
    {
      ROS_ERROR("something wrong with the config file (wheel placement configurations), exiting!");
      exit (-1);
    }

    for (uint i = 0; i < wheel_placements_.size (); i++)
    {
      ROS_DEBUG("wheel %d: angular_pos: %f", i, wheel_placements_[i]);
      wheel_placements_[i] = math::degToRad (wheel_placements_[i]);
    }
  }

  SSLPlayer::~SSLPlayer ()
  {
    ros::shutdown ();
  }

  void
  SSLPlayer::move (const float v_x, const float v_y, const float w)
  {
    for (uint i = 0; i < wheel_vels_.size (); i++)
      wheel_vels_[i] = (sin (wheel_placements_[i]) * v_x - cos (wheel_placements_[i]) * v_y - robot_radius_ * w)
          / wheel_radius_;
  }
}
