/*
 * SpatialFExtractor.cpp
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

#include "al_perception2/feature_extractors/SpatialFExtractor.h"

namespace al
{

  namespace perception
  {

    SpatialFExtractor::SpatialFExtractor (ros::NodeHandle* nh) :
      FeatureExtractor (nh)
    {
      // TODO Auto-generated constructor stub

    }

    SpatialFExtractor::~SpatialFExtractor ()
    {
      // TODO Auto-generated destructor stub
    }

    void
    SpatialFExtractor::extract (al::perception::Entity &entity)
    {
      //      entity.getCollisionObject().poses[0].position
      al_msgs::FeatureVector spatial_features;
      al_msgs::Feature position_x;
      position_x.val_type = al_msgs::Feature::SINGLE_VALUED;
      position_x.type = al_msgs::Feature::POS_X;
      position_x.avg = entity.getCollisionObject ().poses[0].position.x;
      position_x.range_min = -1.0;
      position_x.range_max = 1.0;

      al_msgs::Feature position_y;
      position_y.val_type = al_msgs::Feature::SINGLE_VALUED;
      position_y.type = al_msgs::Feature::POS_Y;
      position_y.avg = entity.getCollisionObject ().poses[0].position.y;
      position_y.range_min = -1.0;
      position_y.range_max = 1.0;

      al_msgs::Feature position_z;
      position_z.val_type = al_msgs::Feature::SINGLE_VALUED;
      position_z.type = al_msgs::Feature::POS_Z;
      position_z.avg = entity.getCollisionObject ().poses[0].position.z;
      position_z.range_min = -1.0;
      position_z.range_max = 1.50;

      geometry_msgs::Quaternion rot = entity.getCollisionObject ().poses[0].orientation;
      tf::Quaternion q;
      tf::quaternionMsgToTF (rot, q);
      double yaw_angle = tf::getYaw (q);
      if (yaw_angle < 0)
        yaw_angle += 2 * M_PI;
      al_msgs::Feature rotation_t;
      rotation_t.val_type = al_msgs::Feature::SINGLE_VALUED;
      rotation_t.type = al_msgs::Feature::ROT_T;
      rotation_t.avg = yaw_angle;
      rotation_t.range_min = 0.0;
      rotation_t.range_max = 2 * M_PI;

      al_msgs::Feature dimension_x;
      dimension_x.val_type = al_msgs::Feature::SINGLE_VALUED;
      dimension_x.type = al_msgs::Feature::DIM_X;
      dimension_x.avg = entity.getCollisionObject ().shapes[0].dimensions[0];
      dimension_x.range_min = 0.0;
      dimension_x.range_max = 1.0;

      al_msgs::Feature dimension_y;
      dimension_y.val_type = al_msgs::Feature::SINGLE_VALUED;
      dimension_y.type = al_msgs::Feature::DIM_Y;
      dimension_y.avg = entity.getCollisionObject ().shapes[0].dimensions[1];
      dimension_y.range_min = 0.0;
      dimension_y.range_max = 1.0;

      al_msgs::Feature dimension_z;
      dimension_z.val_type = al_msgs::Feature::SINGLE_VALUED;
      dimension_z.type = al_msgs::Feature::DIM_Z;
      dimension_z.avg = entity.getCollisionObject ().shapes[0].dimensions[2];
      dimension_z.range_min = 0.0;
      dimension_z.range_max = 1.0;

      al_msgs::Feature object_presence;
      object_presence.val_type = al_msgs::Feature::SINGLE_VALUED;
      object_presence.type = al_msgs::Feature::OBJECT_PRESENCE;
      object_presence.avg = 1.0;
      object_presence.range_min = 0.0;
      object_presence.range_max = 1.0;

      spatial_features.features.resize (8);
      spatial_features.features[0] = position_x;
      spatial_features.features[1] = position_y;
      spatial_features.features[2] = position_z;
      spatial_features.features[3] = rotation_t;
      spatial_features.features[4] = dimension_x;
      spatial_features.features[5] = dimension_y;
      spatial_features.features[6] = dimension_z;
      spatial_features.features[7] = object_presence;

      entity.appendFeatures (spatial_features);
    }
  }
}
