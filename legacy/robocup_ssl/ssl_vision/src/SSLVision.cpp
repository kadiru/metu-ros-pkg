/*
 * SSLVision.cpp
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

#include "ssl_vision/SSLVision.h"

namespace ssl {

SSLVision::SSLVision(ros::NodeHandle* nh) {
  nh_ = nh;

  pub_ = nh_->advertise<ssl_msgs::GlobalState>(ssl::naming::topic::RAW_GLOBAL_STATE, 10);
  int team_capacity = ssl::config::team::TEAM_CAPACITY;

  if(!nh_->getParam("Teams/capacity", team_capacity))
    ROS_WARN("no parameter as <Teams/capacity>");

  field_width_ = ssl::config::game::FIELD_WIDTH;
  if(!nh_->getParam("Field/width", field_width_))
    ROS_WARN("no parameter as <Field/width>");
  field_height_= ssl::config::game::FIELD_HEIGHT;
  if(!nh_->getParam("Field/height", field_height_))
    ROS_WARN("no parameter as <Field/height>");
  field_outer_width_=ssl::config::game::FIELD_OUTER_WIDTH;
  if(!nh_->getParam("Field/outer_width", field_outer_width_))
    ROS_WARN("no parameter as <Field/outer_width>");
  field_outer_height_= ssl::config::game::FIELD_OUTER_HEIGHT;
  if(!nh_->getParam("Field/outer_height", field_outer_height_))
    ROS_WARN("no parameter as <Field/outer_height>");

  ssl_msgs::GlobalRobotState state;
  state.state = ssl::visual::OUT_OF_FOV;
  state.team = ssl::naming::team::BLUE_TEAM;
  global_state_.blue_team.resize (team_capacity, state);
  state.team = ssl::naming::team::YELLOW_TEAM;
  global_state_.yellow_team.resize (team_capacity, state);

  global_state_.header.frame_id =ssl::naming::frame::FIELD;
  global_state_.header.seq = 0;
  global_state_.header.stamp = ros::Time::now();
}

SSLVision::~SSLVision() {
  ros::shutdown ();
}

void
SSLVision::publish()
{
  if(pub_.getNumSubscribers())
    pub_.publish(global_state_);
}

void
SSLVision::init()
{
}

void
SSLVision::run ()
{
  ros::Rate r (1000.0/ssl::config::sim::TIME_STEP);
  while(nh_->ok())
  {
    update();
    publish();
    ros::spinOnce();
    r.sleep();
  }
}
}
