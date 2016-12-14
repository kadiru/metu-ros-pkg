/*
 * SSLRealPlayer.cpp
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

#include "ssl_player/SSLRealPlayer.h"

namespace ssl
{

  SSLRealPlayer::SSLRealPlayer (ros::NodeHandle* nh):SSLPlayer(nh)
  {
    sub_move_ = nh_->subscribe ("move", 10, &SSLRealPlayer::moveCallback, this);
    sub_kick_ = nh_->subscribe ("kick", 10, &SSLRealPlayer::kickCallback, this);
    sub_drib_ = nh_->subscribe ("drib", 10, &SSLRealPlayer::dribCallback, this);
    sub_chip_ = nh_->subscribe ("chip", 10, &SSLRealPlayer::chipCallback, this);

    // TODO: Add anything necessary to initialize I2C communication
    // and handle possible errors here

  }

  SSLRealPlayer::~SSLRealPlayer ()
  {
    // TODO: Add anything necessary to close I2C communication
  }

  void
  SSLRealPlayer::moveCallback (const geometry_msgs::TwistConstPtr msg_move)
  {
    ROS_DEBUG("cmd_move: %f %f %f", cmd_move_.linear.x, cmd_move_.linear.y, cmd_move_.angular.z);
    cmd_move_ = *msg_move;
    move (cmd_move_.linear.x, cmd_move_.linear.y, cmd_move_.angular.z);
  }

  void
  SSLRealPlayer::kickCallback (const ssl_msgs::KickConstPtr msg_kick)
  {
    //TODO: stub
  }

  void
  SSLRealPlayer::dribCallback (const ssl_msgs::DribConstPtr msg_drib)
  {
    //TODO: stub
  }

  void
  SSLRealPlayer::chipCallback (const ssl_msgs::ChipConstPtr msg_chip)
  {
    //TODO: stub
  }

  void
  SSLRealPlayer::move (const float v_x, const float v_y, const float w)
  {
    //this will set wheel_vels_
    SSLPlayer::move(v_x, v_y, w);

    //TODO: convert wheel_vels_ vector to a necessary format to send
    //them via I2C protocol to peripheral uControllers

  }

  void
  SSLRealPlayer::kick (const float strength, const float direction)
  {
    //TODO: stub
  }

  void
  SSLRealPlayer::drib (const float strength)
  {
    //TODO: stub
  }

  void
  SSLRealPlayer::chip (const float strength)
  {
    //TODO: stub
  }

  void
  SSLRealPlayer::run ()
  {
    //TODO: stub
  }

}
