/*
 * speech_test.cpp
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

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "al_acts/TextToSpeechAction.h"

#include "string"

typedef actionlib::SimpleActionClient<al_acts::TextToSpeechAction> SpeechClient;

ros::NodeHandle* nh_;
SpeechClient* speech_client_;

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_unit_tests");
  nh_ = new ros::NodeHandle ();

  speech_client_ = new SpeechClient ("speech_action", true);
  speech_client_->waitForServer ();
  if (speech_client_->isServerConnected ())
    std::cout << "server connected!" << std::endl;
  al_acts::TextToSpeechGoal goal;

  std::string speech_text;
  ros::Rate r (50);
  bool text_entered = false;
  while (nh_->ok ())
  {
    if (!text_entered)
    {
      std::cout << "type a text to get it said" << std::endl;
      std::getline (std::cin, speech_text);
    }
    //    std::cin >> speech_text;
    if (speech_text.size ())
    {
      if (!text_entered)
      {
        goal.text = speech_text;
        speech_client_->sendGoal (goal);
        text_entered = true;
      }
      else
      {
        bool finished_within_time = speech_client_->waitForResult (ros::Duration (10));
        if (!finished_within_time)
        {
          std::cout << "time is over to speak" << std::endl;
          speech_client_->cancelGoal ();
          text_entered = false;
        }
        else
        {
          actionlib::SimpleClientGoalState state = speech_client_->getState ();
          bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
          if (success)
          {
            ROS_INFO("Action finished: %s",state.toString().c_str());
          }
          else
          {
            ROS_INFO("Action failed: %s",state.toString().c_str());
          }
          text_entered = false;
        }
      }
    }
    ros::spinOnce ();
    r.sleep ();
  }

  ros::spin ();

  return 0;
}
