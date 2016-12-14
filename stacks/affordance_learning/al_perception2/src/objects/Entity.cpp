/*
 * Entity.cpp
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

#include "al_perception2/objects/Entity.h"

namespace al
{
  namespace perception
  {
    al_msgs::Entity
    toEntityMsg (const al::perception::Entity &entity)
    {
      al_msgs::Entity msg_entity;
      msg_entity.collision_object = ((al::perception::Entity)entity).getCollisionObject ();
      pcl::toROSMsg (*((al::perception::Entity)entity).getCloudData (), msg_entity.cloud_object);
      msg_entity.feature_vector = ((al::perception::Entity)entity).getFeatures ();

      return msg_entity;
    }

    al::perception::Entity
    fromEntityMsg (const al_msgs::Entity &msg_entity)
    {
      al::perception::Entity entity;
      if (msg_entity.cloud_object.data.size ())
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (msg_entity.cloud_object, *cloud_ptr);
        entity.setCloudData (cloud_ptr);
      }
      entity.setFeatures (msg_entity.feature_vector);
      entity.setCollisionObject (msg_entity.collision_object);

      return entity;
    }

    Entity::Entity ()
    {
      // TODO Auto-generated constructor stub
    }

    Entity::~Entity ()
    {
      // TODO Auto-generated destructor stub
    }

    int
    Entity::getId ()
    {
      return al::facilities::toInt (collision_object_.id);
    }

    void
    Entity::setId (int id)
    {
      collision_object_.id = al::facilities::toString (id);
    }

    al_msgs::FeatureVector
    Entity::getFeatures ()
    {
      return features_;
    }

    void
    Entity::setFeatures (const al_msgs::FeatureVector& features)
    {
      features_ = features;
    }

    void
    Entity::appendFeatures (const al_msgs::FeatureVector& features)
    {
      features_.features.insert (features_.features.end (), features.features.begin (), features.features.end ());
    }

    void
    Entity::cleanFeatures ()
    {
      features_.features.clear ();
    }

    arm_navigation_msgs::CollisionObject
    Entity::getCollisionObject ()
    {
      return collision_object_;
    }

    void
    Entity::setCollisionObject (const arm_navigation_msgs::CollisionObject &collision_object)
    {
      collision_object_ = collision_object;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    Entity::getCloudData ()
    {
      return cloud_data_;
    }

    void
    Entity::setCloudData (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_data)
    {
      cloud_data_ = cluster_data;
    }

    bool
    Entity::visualizable ()
    {
      if (collision_object_.poses.size () > 0 && getId () > 0 && getId () < MAX_N_OBJECTS_IN_ENVIRONMENT)
        return true;
      else
        return false;
    }

    bool
    Entity::processable ()
    {
      if (visualizable () && cloud_data_->points.size () > 0)
        return true;
      else
        return false;
    }
  }
}
