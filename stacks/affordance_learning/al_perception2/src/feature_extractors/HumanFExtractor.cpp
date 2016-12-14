/*
 * HumanFExtractor.cpp
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

#include "al_perception2/feature_extractors/HumanFExtractor.h"

namespace al
{

  namespace perception
  {

    HumanFExtractor::HumanFExtractor (ros::NodeHandle* nh) :
        FeatureExtractor (nh)
    {
      tf_listener_ = new tf::TransformListener ();
    }

    HumanFExtractor::~HumanFExtractor ()
    {
      // TODO Auto-generated destructor stub
    }

    void
    HumanFExtractor::extract (al::perception::Entity &entity)
    {
      bool transformation_healthy = true;

      al_msgs::FeatureVector human_features;
      al_msgs::Feature torso_x;
      al_msgs::Feature torso_y;
      al_msgs::Feature torso_z;
      al_msgs::Feature torso_roll;
      al_msgs::Feature torso_pitch;
      al_msgs::Feature torso_yaw;
      al_msgs::Feature head_roll;
      al_msgs::Feature head_pitch;
      al_msgs::Feature head_yaw;
      al_msgs::Feature human_presence;
      torso_x.type = al_msgs::Feature::HUMAN_TORSO_X;
      torso_y.type = al_msgs::Feature::HUMAN_TORSO_Y;
      torso_z.type = al_msgs::Feature::HUMAN_TORSO_Z;
      torso_roll.type = al_msgs::Feature::HUMAN_TORSO_ROLL;
      torso_pitch.type = al_msgs::Feature::HUMAN_TORSO_PITCH;
      torso_yaw.type = al_msgs::Feature::HUMAN_TORSO_YAW;
      head_roll.type = al_msgs::Feature::HUMAN_HEAD_ROLL;
      head_pitch.type = al_msgs::Feature::HUMAN_HEAD_PITCH;
      head_yaw.type = al_msgs::Feature::HUMAN_HEAD_YAW;
      human_presence.type = al_msgs::Feature::HUMAN_PRESENCE;
      try
      {
        tf_listener_->lookupTransform ("base_footprint", "hat_link", ros::Time (0), transform_head_);
      }
      catch (tf::TransformException ex)
      {
        //        transform_head_.stamp_.
        transformation_healthy = false;
      }
      try
      {
        tf_listener_->lookupTransform ("base_footprint", "torso_1", ros::Time (0), transform_torso_);
      }
      catch (tf::TransformException ex)
      {
        transformation_healthy = false;
      }
      if (transformation_healthy)
      {
        torso_x.avg = transform_torso_.getOrigin ().x ();
        torso_y.avg = transform_torso_.getOrigin ().y ();
        torso_z.avg = transform_torso_.getOrigin ().z ();

//        btScalar r, p, y;
        tfScalar r_, p_, y_;
//        transform_torso_.getBasis ().getRPY (r, p, y);
        transform_torso_.getBasis ().getRPY (r_, p_, y_);
//        torso_roll.avg = r;
//        torso_pitch.avg = p;
//        torso_yaw.avg = y;
        torso_roll.avg = r_;
        torso_pitch.avg = p_;
        torso_yaw.avg = y_;
//        transform_head_.getBasis ().getRPY (r, p, y);
        transform_head_.getBasis ().getRPY (r_, p_, y_);
//        head_roll.avg = r;
//        head_pitch.avg = p;
//        head_yaw.avg = y;
        head_roll.avg = r_;
        head_pitch.avg = p_;
        head_yaw.avg = y_;
        //TODO: this may change for PR2
        if (torso_z.avg < 0)
          human_presence.avg = 1.0;
        else
          human_presence.avg = 0.0;
      }
      else
      {
        //either human doesn't exist or not perceived correctly
        torso_x.avg = 0;
        torso_y.avg = 0;
        torso_z.avg = 0;

        torso_roll.avg = 0;
        torso_pitch.avg = 0;
        torso_yaw.avg = 0;
        head_roll.avg = 0;
        head_pitch.avg = 0;
        head_yaw.avg = 0;
        human_presence.avg = 0;
      }

      torso_x.range_min = -3.0;
      torso_x.range_max = 3.0;

      human_features.features.resize (10);
      human_features.features[0] = torso_x;
      human_features.features[1] = torso_y;
      human_features.features[2] = torso_z;
      human_features.features[3] = torso_roll;
      human_features.features[4] = torso_pitch;
      human_features.features[5] = torso_yaw;
      human_features.features[6] = head_roll;
      human_features.features[7] = head_pitch;
      human_features.features[8] = head_yaw;
      human_features.features[9] = human_presence;

      for (uint i = 0; i < human_features.features.size (); i++)
        human_features.features[i].val_type = al_msgs::Feature::SINGLE_VALUED;

      entity.appendFeatures (human_features);
    }
  }
}
