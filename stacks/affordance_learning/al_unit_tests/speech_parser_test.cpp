/*
 * speech_parser_test.cpp
 * Copyright (c) 2012, Kadir Firat Uyanik, Kovan Research Lab, METU
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

#include "al_msgs/Speech.h"

#include "ros/ros.h"

ros::NodeHandle* nh_;
ros::Subscriber sub;

void
speechCallback (al_msgs::Speech::ConstPtr speech_msg)
{
  std::cout << "behavior  :\n" << speech_msg->behavior << std::endl;
  std::cout << "effect    :\n" << speech_msg->effect << std::endl;
  std::cout << "adjective :\n" << speech_msg->adjective << std::endl;
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_unit_tests");
  nh_ = new ros::NodeHandle ();

  sub = nh_->subscribe ("speech_command", 1, &speechCallback);

  ros::Rate r (50);
  while (nh_->ok ())
  {
    ros::spinOnce ();
    r.sleep ();
  }
  return 0;
}
