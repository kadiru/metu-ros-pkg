/*
 * SSLWSimVision.cpp
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

#include "ssl_vision/SSLWSimVision.h"

namespace ssl
{

  SSLWSimVision::SSLWSimVision (ros::NodeHandle* nh) :
    SSLVision (nh)
  {

    init ();

    step (ssl::config::sim::TIME_STEP);
  }

  SSLWSimVision::~SSLWSimVision ()
  {
    // TODO Auto-generated destructor stub
  }

  void
  SSLWSimVision::update ()
  {
    global_state_.header.stamp = ros::Time::now ();
    global_state_.header.frame_id = ssl::naming::frame::FIELD;

    updateBall ();

    for (uint8_t j = 0; j < ssl::config::team::TEAM_CAPACITY; j++)
      updateRobot (ssl::naming::team::BLUE_TEAM, j);
    for (uint8_t j = 0; j < ssl::config::team::TEAM_CAPACITY; j++)
      updateRobot (ssl::naming::team::YELLOW_TEAM, j);

    if (step (ssl::config::sim::TIME_STEP) == -1)
      exit (-1);
  }

  void
  SSLWSimVision::updateBall ()
  {
    webots::Node* entity_node = NULL;
    entity_node = getFromDef ("BALL");
    global_state_.balls.clear ();
    if (entity_node != NULL)
    {
      ssl_msgs::BallState ball;
      ball.position.x = entity_node->getField ("translation")->getSFVec3f ()[0];
      ball.position.y = entity_node->getField ("translation")->getSFVec3f ()[1];
      ball.position.z = entity_node->getField ("translation")->getSFVec3f ()[2];

      ball.confidence = 1.0;
      global_state_.balls.push_back (ball);
    }
  }

  void
  SSLWSimVision::updateRobot (const std::string team, const uint8_t& id)
  {
    webots::Node* entity_node = NULL;

    ssl_msgs::GlobalRobotState state;
    state.confidence = 1.0;
    state.id = id;
    entity_node = getFromDef (ssl::naming::createRobotName (team, id));
    //TODO: check whether a robot is in no_move state
    //e.g. no move for 5 seconds, then assign robot.state=NO_MOVE
    if (entity_node != NULL)
    {
      state.pose.x = entity_node->getField ("translation")->getSFVec3f ()[0];
      state.pose.y = entity_node->getField ("translation")->getSFVec3f ()[1];
      //if z axis is pointing in the -1 direction
      state.pose.theta = (entity_node->getField ("rotation")->getSFRotation ()[3]
          * ssl::math::sign (entity_node->getField ("rotation")->getSFRotation ()[2]));
      //TODO: map the orientation between [-PI,PI] or [0, 2*PI]

      if (fabs (state.pose.x) > field_width_ / 2.0 || fabs (state.pose.y)
          > field_height_ / 2.0)
        state.state = ssl::visual::OUTSIDE_FIELD;
      else
        state.state = ssl::visual::INSIDE_FIELD;
    }
    else
      state.state = ssl::visual::OUT_OF_FOV;
    if (team == ssl::naming::team::YELLOW_TEAM)
    {
      state.team = ssl::naming::team::YELLOW_TEAM;
      global_state_.yellow_team[id] = state;
    }
    else
    {
      state.team = ssl::naming::team::BLUE_TEAM;
      global_state_.blue_team[id] = state;
    }
  }

}
