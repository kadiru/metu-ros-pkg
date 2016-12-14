/*
 * human_to_robot.cpp
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

#include "al_msgs/Speech.h"
#include "al_msgs/Shape.h"
#include "al_msgs/Spec.h"
#include "al_utils/al_utils.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "string"

std::string robot_name;
std::string speech_topic_name = "/human/output";
std::string speech_msg_;
bool speech_msg_rcvd_ = false;
bool new_speech_msg_rcvd_ = false;
ros::Time t_robot_addressed_;

void
speechCallback (std_msgs::String::ConstPtr msg);

bool
parseTextToCommand (const std::string& speech_msg, al_msgs::Speech& speech_cmd);

bool
verbTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd);

bool
adjTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd);

bool
nounTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd);

bool
effectTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_speech");
  ros::NodeHandle nh ("~");

  if (!nh.getParam ("robot_name", robot_name))
    ROS_WARN("no such parameter as <robot_name>");

  //  speech_topic_name = "/" + robot_name + "/output";
  ROS_DEBUG("%s", robot_name.c_str());

  ros::Subscriber sub;
  ros::Publisher pub;

  sub = nh.subscribe (speech_topic_name, 2, &speechCallback);
  pub = nh.advertise<al_msgs::Speech> ("/speech_command", 1);

  ROS_INFO("waiting for a speech command to be received!");
  while (nh.ok () && !speech_msg_rcvd_)
  {
    ros::spinOnce ();
  }
  ROS_INFO("first speech command received!");

  while (nh.ok ())
  {
    if (new_speech_msg_rcvd_)
    {
      new_speech_msg_rcvd_ = false;
      al_msgs::Speech speech_cmd;
      bool parsed = true;//parseTextToCommand (speech_msg_, speech_cmd);
      if (parsed/* && pub.getNumSubscribers()*/)
      {
        std::cout << speech_cmd << std::endl;
        pub.publish (speech_cmd);
      }
    }
    ros::spinOnce ();
  }
  return 0;
}

void
speechCallback (std_msgs::String::ConstPtr msg)
{
  if (!speech_msg_rcvd_)
    speech_msg_rcvd_ = true;
  std::cout << msg->data << std::endl;
  new_speech_msg_rcvd_ = true;
  speech_msg_ = msg->data;
}

bool
parseTextToCommand (const std::string& speech_msg, al_msgs::Speech& speech_cmd)
{
  bool robot_addressed = false;
  speech_cmd.behavior.behavior = -1;
  speech_cmd.behavior.arg = -1;
  speech_cmd.effect.effect = -1;
  speech_cmd.effect.prob = 1.0;

  //check if the robot is addressed
  int robot_name_index = speech_msg.find (robot_name);
  if (robot_name_index != (int)speech_msg.npos)
    t_robot_addressed_ = ros::Time::now ();

  bool testVerb = false, testAdj = false, testNoun = false, testEffect = false;
  //if robots name is mentioned at most 5 sec ago, it is addressed in the current interaction period
  if (ros::Time::now () - t_robot_addressed_ < ros::Duration (5.0))
    robot_addressed = true;

  //the rule is: <robot_name> <command> <...> <argument>
  if (robot_addressed)
  {
    std::string rest_of_the_sentence = speech_msg_.substr (robot_name_index);
    std::cout << "rest of the sentence" << std::endl;
    testVerb = verbTest (rest_of_the_sentence, speech_cmd);
    testEffect = effectTest (rest_of_the_sentence, speech_cmd);
    testAdj = adjTest (rest_of_the_sentence, speech_cmd);
    testNoun = nounTest (rest_of_the_sentence, speech_cmd);

    if (testVerb && testNoun)
    {
      // sentence is correct. go with it.
      speech_cmd.behavior.arg--;
      speech_cmd.effect.arg = -1;
      return true;
    }
    else if (testEffect)
    {
      speech_cmd.effect.arg--;
      speech_cmd.behavior.arg = -1;
      return true;
    }
    return false;
  }
  else
  {
    return false;
  }
  //if behavior command is detected, effect command will be discarded
  //this is necessary since "object n" corresponds to "object n-1"
  speech_cmd.behavior.arg--;
  speech_cmd.effect.arg--;
  if (speech_cmd.behavior.arg < 0) // necessary precaution to avoid wrong outputs ('down', 'dont', etc.)
    return false;
  else
    return true;

}

bool
verbTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd)
{
  speech_cmd.behavior.behavior = -1;
  if (speech_msg.find ("push right") != std::string::npos || speech_msg.find ("pushed right") != std::string::npos)//action
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_RGT;
    ROS_INFO("%s push RIGHT", robot_name.c_str());
  }
  else if (speech_msg.find ("push left") != std::string::npos || speech_msg.find ("pushed left") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_LFT;
    ROS_INFO("%s push LEFT", robot_name.c_str());
  }
  else if (speech_msg.find ("push forward") != std::string::npos || speech_msg.find ("pushed forward")
      != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_FWD;
    ROS_INFO("%s push FORWARD", robot_name.c_str());
  }
  else if (speech_msg.find ("push backward") != std::string::npos || speech_msg.find ("pushed backward")
      != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_BWD;
    ROS_INFO("%s push BACKWARD", robot_name.c_str());
  }
  else if (speech_msg.find ("show") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::SHOW;
    ROS_INFO("%s show", robot_name.c_str());
  }
  else if (speech_msg.find ("home") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::HOME;
    ROS_INFO("%s home", robot_name.c_str());
  }
  //    else if (speech_msg.find ("grasp") == robot_name_index + robot_name.size () + 1)
  //    {
  //      speech_cmd.behavior.behavior = al_msgs::Behavior::GRASP;
  //      ROS_INFO("%s grasp", robot_name.c_str());
  //    }
  else if (speech_msg.find ("reach") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::REACH;
    ROS_INFO("%s reach", robot_name.c_str());
  }
  else if (speech_msg.find ("take") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::TAKE;
    ROS_INFO("%s take", robot_name.c_str());
  }
  else if (speech_msg.find ("give") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::GIVE;
    ROS_INFO("%s give", robot_name.c_str());
  }
  else if (speech_msg.find ("release") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::RELEASE;
    ROS_INFO("%s release", robot_name.c_str());
  }
  else if (speech_msg.find ("continue") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::CONTINUE;
    ROS_INFO("%s continue", robot_name.c_str());
  }
  else if (speech_msg.find ("tuck arms") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::TUCK_ARMS;
    ROS_INFO("%s tuck arms", robot_name.c_str());
  }
  else if (speech_msg.find ("cover") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::COVER;
    ROS_INFO("%s cover", robot_name.c_str());
  }
  else if (speech_msg.find ("call") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::CALL;
    ROS_INFO("%s call", robot_name.c_str());
  }
  else if (speech_msg.find ("grasp top") != std::string::npos || speech_msg.find ("grasped top") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::GRASP_TOP;
    ROS_INFO("%s grasp TOP", robot_name.c_str());
  }
  else if (speech_msg.find ("grasp side") != std::string::npos || speech_msg.find ("grasped side") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::GRASP_SIDE;
    ROS_INFO("%s grasp SIDE", robot_name.c_str());
  }
  else if (speech_msg.find ("push top right") != std::string::npos || speech_msg.find ("pushed top right")
      != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_TOP_RGT;
    ROS_INFO("%s push TOP RIGHT", robot_name.c_str());
  }
  else if (speech_msg.find ("push top left") != std::string::npos || speech_msg.find ("pushed top left")
      != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_TOP_LFT;
    ROS_INFO("%s push TOP LEFT", robot_name.c_str());
  }
  else if (speech_msg.find ("push top forward") != std::string::npos || speech_msg.find ("pushed top forward")
      != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_TOP_FWD;
    ROS_INFO("%s push TOP FORWARD", robot_name.c_str());
  }
  else if (speech_msg.find ("push top backward") != std::string::npos || speech_msg.find ("pushed top backward")
      != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::PUSH_TOP_BWD;
    ROS_INFO("%s push TOP BACKWARD", robot_name.c_str());
  }
  else if (speech_msg.find ("point") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::POINT;
    ROS_INFO("%s point", robot_name.c_str());
  }
  else if (speech_msg.find ("hide") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::HIDE;
    ROS_INFO("%s hide", robot_name.c_str());
  }
  else if (speech_msg.find ("cancel") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::CANCEL;
    ROS_INFO("%s cancel", robot_name.c_str());
  }
  else if (speech_msg.find ("stop") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::STOP;
    ROS_INFO("%s stop", robot_name.c_str());
  }
  else if (speech_msg.find ("lift") != std::string::npos || speech_msg.find ("lifted") != std::string::npos)
  {
    speech_cmd.behavior.behavior = al_msgs::Behavior::GRASP_TOP;
    ROS_INFO("%s lift", robot_name.c_str());
  }
  return speech_cmd.behavior.behavior != -1;
}
bool
adjTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd)
{
  bool adjFound = false;
  for (uint i = 0; i <= al_msgs::Shape::MAX_SHAPE_INDEX; i++)
  {
    if (speech_msg.find (al::facilities::specEnumToString (i)) != std::string::npos)
    {
      speech_cmd.adjective.spec = (int8_t)i;
      adjFound = true;
      break;
    }
  }

  return adjFound;
}

bool
nounTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd)
{
  bool nounFound = false;
  for (uint i = 0; i <= al_msgs::Shape::MAX_SHAPE_INDEX; i++)
  {
    if (speech_msg.find (al::facilities::shapeEnumToString (i)) != std::string::npos)
    {
      speech_cmd.behavior.arg = i;
      nounFound = true;
      break;
    }
  }

  return nounFound;

  /*
   //now, extract argument (only nine object names(ids) are supported for the moment)
   if (speech_msg.find ("object one") != std::string::npos)
   {
   speech_cmd.behavior.arg = 1;
   speech_cmd.effect.arg = 1; // we can use this for adjective.
   ROS_INFO("object one");
   }
   else if (speech_msg.find ("object two") != std::string::npos)
   {
   speech_cmd.behavior.arg = 2;
   speech_cmd.effect.arg = 2;
   ROS_INFO("object two");
   }
   else if (speech_msg.find ("object three") != std::string::npos)
   {
   speech_cmd.behavior.arg = 3;
   speech_cmd.effect.arg = 3;
   ROS_INFO("object three");
   }
   else if (speech_msg.find ("object four") != std::string::npos)
   {
   speech_cmd.behavior.arg = 4;
   speech_cmd.effect.arg = 4;
   ROS_INFO("object four");
   }
   else if (speech_msg.find ("object five") != std::string::npos)
   {
   speech_cmd.behavior.arg = 5;
   speech_cmd.effect.arg = 5;
   ROS_INFO("object five");
   }
   else if (speech_msg.find ("object six") != std::string::npos)
   {
   speech_cmd.behavior.arg = 6;
   speech_cmd.effect.arg = 6;
   ROS_INFO("object six");
   }
   else if (speech_msg.find ("object seven") != std::string::npos)
   {
   speech_cmd.behavior.arg = 7;
   speech_cmd.effect.arg = 7;
   ROS_INFO("object seven");
   }
   else if (speech_msg.find ("object eight") != std::string::npos)
   {
   speech_cmd.behavior.arg = 8;
   speech_cmd.effect.arg = 8;
   ROS_INFO("object eight");
   }
   else if (speech_msg.find ("object nine") != std::string::npos)
   {
   speech_cmd.behavior.arg = 9;
   speech_cmd.effect.arg = 9;
   ROS_INFO("object nine");
   }*/
}
bool
effectTest (const std::string& speech_msg, al_msgs::Speech& speech_cmd)
{
  //now extract effect information

  speech_cmd.effect.effect = -1;
  if (speech_msg.find ("disappeared") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::DISAPPEARED;
    ROS_INFO("effect disappeared");
  }
  else if (speech_msg.find ("moved left") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::MOVED_LEFT;
    ROS_INFO("effect moved left");
  }
  else if (speech_msg.find ("moved right") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::MOVED_RIGHT;
    ROS_INFO("effect moved right");
  }
  else if (speech_msg.find ("moved forward") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::MOVED_FORWARD;
    ROS_INFO("effect moved forward");
  }
  else if (speech_msg.find ("moved backward") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::MOVED_BACKWARD;
    ROS_INFO("effect moved backward");
  }
  else if (speech_msg.find ("reached") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::REACHED;
    ROS_INFO("effect reached");
  }
  else if (speech_msg.find ("taken") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::TAKEN;
    ROS_INFO("effect taken");
  }
  else if (speech_msg.find ("given") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::GIVEN;
    ROS_INFO("effect given");
  }
  else if (speech_msg.find ("no change") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::NO_CHANGE;
    ROS_INFO("effect no change");
  }
  else if (speech_msg.find ("vanished") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::VANISHED;
    ROS_INFO("effect vanished");
  }
  else if (speech_msg.find ("rotated clockwise") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::ROTATED_CW;
    ROS_INFO("effect rotated clockwise");
  }
  else if (speech_msg.find ("rotated counter clockwise") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::ROTATED_CCW;
    ROS_INFO("effect rotated counter clockwise");
  }

  else if (speech_msg.find ("toppled right") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::TOPPLED_RGT;
    ROS_INFO("effect toppled right");
  }
  else if (speech_msg.find ("toppled left") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::TOPPLED_LFT;
    ROS_INFO("effect toppled left");
  }
  else if (speech_msg.find ("toppled backward") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::TOPPLED_BWD;
    ROS_INFO("effect toppled backward");
  }
  else if (speech_msg.find ("toppled forward") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::TOPPLED_FWD;
    ROS_INFO("effect toppled forward");
  }
  else if (speech_msg.find ("rolled right") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::ROLLED_RIGHT;
    ROS_INFO("effect rolled right");
  }
  else if (speech_msg.find ("rolled left") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::ROLLED_LEFT;
    ROS_INFO("effect rolled left");
  }
  else if (speech_msg.find ("rolled forward") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::ROLLED_FORWARD;
    ROS_INFO("effect rolled forward");
  }
  else if (speech_msg.find ("rolled backward") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::ROLLED_BACKWARD;
    ROS_INFO("effect rolled backward");
  }
  else if (speech_msg.find ("grasped") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::GRASPED;
    ROS_INFO("effect grasped");
  }
  else if (speech_msg.find ("acquired") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::ACQUIRED;
    ROS_INFO("effect acquired");
  }
  else if (speech_msg.find ("released") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::RELEASED;
    ROS_INFO("effect released");
  }
  else if (speech_msg.find ("got closer") != speech_msg.npos)
  {
    speech_cmd.effect.effect = al_msgs::Effect::GOT_CLOSER;
    ROS_INFO("effect got closer");
  }
  return speech_cmd.effect.effect >= 0;
}

