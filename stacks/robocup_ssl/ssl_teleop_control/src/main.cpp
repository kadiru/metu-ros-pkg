/*
 * main.cpp
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

#include "ros/ros.h"
#include "ros/console.h"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

#define PI 3.141562

class SSLRobotTeleop
{
public:
  SSLRobotTeleop (ros::NodeHandle& n);

  void
  run ();

private:
  //  ros::NodeHandle nh_;
  ros::Publisher pub_vel_cmd_;
  ros::Subscriber sub_joy_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::Twist rcvd_cmd_vel_;

  int axis_vx, axis_vy, axis_vw;

  void
  joyCb (const sensor_msgs::Joy::ConstPtr& joy_msg);
};

SSLRobotTeleop::SSLRobotTeleop (ros::NodeHandle& n) //:
//    nh_ (n)
{
  //  sub_joy_ = nh_.subscribe ("joy", 10, &SSLRobotTeleop::joyCb, this);
  //  pub_vel_cmd_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 10);

  //  nh_.param ("axis_vx", axis_vx, 3);
  //  nh_.param ("axis_vw", axis_vw, 0);
  //  nh_.param ("axis_vy", axis_vy, 2);

  sub_joy_ = n.subscribe ("joy", 10, &SSLRobotTeleop::joyCb, this);
  pub_vel_cmd_ = n.advertise<geometry_msgs::Twist> ("cmd_vel", 10);

  n.param ("axis_vx", axis_vx, 1);
  n.param ("axis_vw", axis_vw, 3);
  n.param ("axis_vy", axis_vy, 0);
}

void
SSLRobotTeleop::joyCb (const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if ((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size ()))
    rcvd_cmd_vel_.linear.x = joy_msg->axes[axis_vx] * 3.0; //max 3m/s
  else
    rcvd_cmd_vel_.linear.x = 0.0;

  if ((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size ()))
    rcvd_cmd_vel_.linear.y = joy_msg->axes[axis_vy] * 3.0; //max 3m/s
  else
    rcvd_cmd_vel_.linear.y = 0.0;

  if ((axis_vw >= 0) && (((unsigned int)axis_vw) < joy_msg->axes.size ()))
    rcvd_cmd_vel_.angular.z = joy_msg->axes[axis_vw] * 2 * PI; //max 2PI/s
  else
    rcvd_cmd_vel_.angular.z = 0.0;

  ROS_DEBUG("%f %f %f\n", rcvd_cmd_vel_.linear.x, rcvd_cmd_vel_ .linear.y, rcvd_cmd_vel_ .angular.z);
}

void
SSLRobotTeleop::run ()
{
  ros::Rate ctrl_loop_rate (60); //60 Hz
  while (ros::ok ())
  {
    cmd_vel_ = rcvd_cmd_vel_;
    pub_vel_cmd_.publish (cmd_vel_);
    //    std::cout<<cmd_vel_<<std::endl;
    ros::spinOnce ();
    ctrl_loop_rate.sleep ();
  }
}

int
main (int argc, char**argv)
{
  ros::init (argc, argv, "ssl_robot_teleop");
  ros::NodeHandle nh;
  ROSCONSOLE_AUTOINIT;

  SSLRobotTeleop ssl_robot_teleop (nh);
  ssl_robot_teleop.run ();

  return 0;
}

