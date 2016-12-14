/*
 * Entity.h
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

#ifndef ENTITY_H_
#define ENTITY_H_

#include "arm_navigation_msgs/CollisionObject.h"

#include "al_msgs/FeatureVector.h"
#include "al_msgs/Entity.h"
#include "al_utils/al_utils.h"


namespace al
{
  namespace perception
  {
    class Entity;

    al_msgs::Entity
    toEntityMsg (const al::perception::Entity &entity);

    al::perception::Entity
    fromEntityMsg (const al_msgs::Entity &msg_entity);

    class Entity
    {
    public:
      Entity ();

      virtual
      ~Entity ();

      int
      getId ();

      void
      setId (int id);

      al_msgs::FeatureVector
      getFeatures ();

      void
      setFeatures (const al_msgs::FeatureVector& features);

      void
      appendFeatures (const al_msgs::FeatureVector& features);

      void
      cleanFeatures ();

      arm_navigation_msgs::CollisionObject
      getCollisionObject ();

      void
      setCollisionObject (const arm_navigation_msgs::CollisionObject &collision_object);

      pcl::PointCloud<pcl::PointXYZ>::Ptr
      getCloudData ();

      void
      setCloudData (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_data);

      bool
      visualizable ();

      bool
      processable ();

    protected:
      arm_navigation_msgs::CollisionObject collision_object_;
      al_msgs::FeatureVector features_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_;
    };

  }

}

#endif /* ENTITY_H_ */
