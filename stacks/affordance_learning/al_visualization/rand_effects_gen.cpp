/*
 * rand_effects_gen.cpp
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

#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"

#include "al_msgs/AffordancesComp.h"
#include "al_msgs/Affordances.h"
#include "al_utils/al_utils.h"

const uint TOP_N_EFFECTS = 5;

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_rand_effects_gen");
  ros::NodeHandle* nh_;
  nh_ = new ros::NodeHandle ();

  ros::Publisher pub_affordances_;
  al_msgs::Affordances affordances_;

  pub_affordances_ = nh_->advertise<al_msgs::Affordances> ("/affordances", 1);

  std::map<float, al_msgs::Effect> map_effects;
  std::map<float, al_msgs::Effect>::iterator it;

  ros::Rate r (0.2);
  while (nh_->ok ())
  {

    //generater random effect probabilities
    for (int8_t i = 0; i <= al_msgs::Effect::MAX_EFFECT_INDEX; i++)
    {
      al_msgs::Effect effect;
      effect.effect = i;
      effect.prob = al::math::fRand (0.0, 1.0);
      map_effects[effect.prob] = effect;
    }

    //convert into an affordance message and publish
    it = map_effects.end ();
    uint n_effects = 0;
    affordances_.effects.resize (TOP_N_EFFECTS);
    while (it != map_effects.begin () && n_effects < TOP_N_EFFECTS)
    {
      --it;
      affordances_.effects[n_effects] = it->second;
      n_effects++;
    }

    affordances_.object.id = al::facilities::toString (1);
    geometry_msgs::Pose pose;
    //    pose.position.x = al::math::fRand (-0.7, -0.2);
    pose.position.x = al::math::fRand (0.5, 1.0);
    pose.position.y = al::math::fRand (-0.4, 0.4);
    pose.position.z = al::math::fRand (0.58, 0.64);
    affordances_.object.poses.resize (1);
    affordances_.object.poses[0] = pose;
    arm_navigation_msgs::Shape shape;
    shape.dimensions.resize (3);
    shape.dimensions[0] = 0.05;
    shape.dimensions[1] = 0.05;
    shape.dimensions[2] = (pose.position.z - 0.56) * 2;
    affordances_.object.shapes.resize (1);
    affordances_.object.shapes[0] = shape;

    pub_affordances_.publish (affordances_);
    map_effects.clear ();

    ros::spinOnce ();
    r.sleep ();
  }

}
