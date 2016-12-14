/*
 * SSLPlayer.h
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

#ifndef SSLPLAYER_H_
#define SSLPLAYER_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "string"
#include "vector"

#include "ssl_msgs/Chip.h"
#include "ssl_msgs/Kick.h"
#include "ssl_msgs/Drib.h"
#include "ssl_msgs/Stat.h"
#include "ssl_utils/ssl_utils.h"

namespace ssl
{

  class SSLPlayer
  {
  public:
    SSLPlayer (ros::NodeHandle* nh);
    virtual
    ~SSLPlayer ();

    virtual void
    run ()=0;

  protected:
    //ros
    ros::NodeHandle* nh_;
    ros::Subscriber sub_move_;
    ros::Subscriber sub_kick_;
    ros::Subscriber sub_drib_;
    ros::Subscriber sub_chip_;
    ros::Publisher pub_stat_;

    geometry_msgs::Twist cmd_move_;
    ssl_msgs::Kick cmd_kick_;
    ssl_msgs::Drib cmd_drib_;
    ssl_msgs::Chip cmd_chip_;

    //specific
    int id_;
    std::string team_;

    //run
    std::vector<float> wheel_vels_;

    //config
    std::vector<float> wheel_placements_;
    double wheel_radius_;
    double robot_radius_;

    virtual void
    moveCallback (const geometry_msgs::TwistConstPtr msg_move)=0;

    virtual void
    kickCallback (const ssl_msgs::KickConstPtr msg_kick)=0;

    virtual void
    dribCallback (const ssl_msgs::DribConstPtr msg_drib)=0;

    virtual void
    chipCallback (const ssl_msgs::ChipConstPtr msg_chip)=0;

    virtual void
    move (const float v_x, const float v_y, const float w);

    virtual void
    kick (const float strength, const float direction = 0)=0;

    virtual void
    drib (const float strength)=0;

    virtual void
    chip (const float strength)=0;
  };

}

#endif /* SSLPLAYER_H_ */
