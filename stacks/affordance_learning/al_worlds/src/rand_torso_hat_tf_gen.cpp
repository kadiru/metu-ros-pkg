/*
 * rand_torso_hat_tf_gen.cpp
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

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"

#include "al_utils/al_utils.h"

int
main (int argc, char** argv)
{
  //ros initializations
  ros::init (argc, argv, "al_rand_torso_hat_pub");
  ros::NodeHandle n;
  static tf::TransformBroadcaster br;
  tf::Transform tf_base_to_torso;
  tf::Transform tf_torso_to_head;
  float torso_x, torso_y, torso_z, torso_roll, torso_pitch, torso_yaw, head_roll, head_pitch, head_yaw;

  ros::Rate r (10);
  while (n.ok ())
  {
    torso_x = al::math::fRand (1.20, 4.0);
    torso_y = al::math::fRand (-1.0, 1.0);
    torso_z = al::math::fRand (0.40, 0.80);
    //roll between -20 to 20 degrees
    torso_roll = al::math::fRand (-M_PI / 9, M_PI / 9);
    //pitch between -20 to 30 degrees
    torso_pitch = al::math::fRand (-M_PI / 9, M_PI / 6);
    //yaw between -90 to 90
    torso_yaw = al::math::fRand (-M_PI_2, M_PI_2);
    tf_base_to_torso.setOrigin (tf::Vector3 (torso_x, torso_y, torso_z));
    tf::Quaternion q;
    q.setRPY (torso_roll, torso_pitch, torso_yaw);
    tf_base_to_torso.setRotation (q);
    //currently head is same as the body
    head_roll = 0;
    head_pitch = 0;
    head_yaw = 0;
    q.setRPY (head_roll, head_pitch, head_yaw);
    tf_torso_to_head.setRotation (q);

    br.sendTransform (tf::StampedTransform (tf_base_to_torso, ros::Time::now (), "/base_footprint", "/torso_1"));
    br.sendTransform (tf::StampedTransform (tf_torso_to_head, ros::Time::now (), "/torso_1", "/hat_link"));

    std::cout << "torso :" << torso_x << " " << torso_y << " " << torso_z << " " << torso_roll << " " << torso_pitch
        << " " << torso_yaw << std::endl;
    std::cout << "head :" << head_roll << " " << head_pitch << " " << head_yaw << std::endl;

    ros::spinOnce ();
    r.sleep ();
  }
  return 0;
}
