/*
 * wiimote_state_pub.cpp
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

#include "ros/ros.h"
#include "wiimote/State.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Joy.h"

#include "al_utils/al_utils.h"

ros::NodeHandle* nh_;
ros::Publisher pub_wiimote_state_;
ros::Subscriber sub_wiimote_state_;
tf::Vector3 integrated_gyro_data_;

float yaw_ = 0.0;
bool first_time_ = true;
ros::Time t_prev_;

void
wiimoteStateCallback (wiimote::State::ConstPtr wiimote_state);

void
wiimoteJoyStateCallback (sensor_msgs::Joy::ConstPtr wiimote_joy_state);

void
integrateGyroData (const tf::Vector3 gyro_data, ros::Time t_curr, ros::Time t_prev, tf::Vector3 &integrated_gyro_data);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "wiimote_state_pub");

  nh_ = new ros::NodeHandle ();

  std_srvs::EmptyRequest req;
  std_srvs::EmptyResponse res;
  if (!ros::service::call ("/imu/calibrate", req, res))
  {
    ROS_WARN("calibration service call failed, exiting!");
    return 0;
  }

  //  sub_wiimote_state_ = nh_->subscribe ("/wiimote/state", 1, &wiimoteStateCallback);
  sub_wiimote_state_ = nh_->subscribe ("/joy", 1, &wiimoteJoyStateCallback);

  ros::Rate r (100.0);
  //  ros::Time t_prev = ros::Time::now ();
  while (nh_->ok ())
  {
    //    if (ros::Time::now () - t_prev > ros::Duration (5.0))
    //    {
    //      t_prev = ros::Time::now ();
    //      std_srvs::EmptyRequest req;
    //      std_srvs::EmptyResponse res;
    //      if (!ros::service::call ("/imu/calibrate", req, res))
    //      {
    //        ROS_WARN("calibration service call failed, exiting!");
    //        return 0;
    //      }
    //    }
    ros::spinOnce ();
    r.sleep ();
  }

  return 0;
}

void
integrateGyroData (const tf::Vector3 gyro_data, ros::Time t_curr, ros::Time t_prev, tf::Vector3 &integrated_gyro_data)
{
  std::cout << "now  : " << t_curr << std::endl;
  std::cout << "prev : " << t_prev << std::endl;
  float t_diff = al::math::fPrecision ((t_curr - t_prev), 9);//with ns precision
  std::cout << "t_diff: " << t_diff << std::endl;
  std::cout << gyro_data.x () << "\t" << gyro_data.y () << "\t" << gyro_data.z () << "\n";
  integrated_gyro_data += gyro_data * t_diff;
  std::cout << integrated_gyro_data.x () << "\t" << integrated_gyro_data.y () << "\t" << integrated_gyro_data.z ()
      << "\n";
}

void
wiimoteStateCallback (wiimote::State::ConstPtr wiimote_state)
{
  if (first_time_)
  {
    first_time_ = false;
    t_prev_ = wiimote_state->header.stamp;
    return;
  }

  tf::Vector3 gyro_data;
  tf::vector3MsgToTF (wiimote_state->angular_velocity_zeroed, gyro_data);
  integrateGyroData (gyro_data, wiimote_state->header.stamp, t_prev_, integrated_gyro_data_);
  t_prev_ = wiimote_state->header.stamp;
}

void
wiimoteJoyStateCallback (sensor_msgs::Joy::ConstPtr wiimote_joy_state)
{
  if (first_time_)
  {
    first_time_ = false;
    t_prev_ = wiimote_joy_state->header.stamp;
    return;
  }

  t_prev_ = wiimote_joy_state->header.stamp;
  tf::Vector3 gyro_data;
  gyro_data.setValue (wiimote_joy_state->axes[3], wiimote_joy_state->axes[4], wiimote_joy_state->axes[5]);
  integrateGyroData (gyro_data, wiimote_joy_state->header.stamp, t_prev_, integrated_gyro_data_);
  t_prev_ = wiimote_joy_state->header.stamp;
}

