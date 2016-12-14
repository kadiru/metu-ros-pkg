/*
 * camera_state_pub.cpp
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
#include <tf/transform_broadcaster.h>

const std::string BASE_FOOTPRINT_FRAME = "base_footprint";
//const std::string CAMERA_DEPTH_FRAME = "camera_link";
const std::string CAMERA_DEPTH_FRAME = "camera_depth_frame";

ros::NodeHandle* nh_;

int
main (int argc, char* argv[])
{
  ros::init (argc, argv, "kinect_state_pub");
  nh_ = new ros::NodeHandle ();

  static tf::TransformBroadcaster br;

  //  ros::WallRate r(50);
  ros::Rate r (50);
  while (nh_->ok ())
  {
    tf::StampedTransform transform;

    transform.frame_id_ = BASE_FOOTPRINT_FRAME;
    transform.child_frame_id_ = CAMERA_DEPTH_FRAME;

    transform.stamp_ = ros::Time::now ();
    transform.setOrigin (tf::Vector3 (0.5, 0, 0.8));
    tf::Quaternion quat;
    quat.setRPY (0, 0, 0);
    transform.setRotation (quat);
    br.sendTransform (transform);

    ros::spinOnce ();
    r.sleep ();
    std::cout << "asd" << std::endl;
  }
  return 1;
}
