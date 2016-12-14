/*
 * Memory.h
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

#ifndef MEMORY_H_
#define MEMORY_H_

#include "ros/ros.h"

#include "vector"

#include "al_utils/al_utils.h"
#include "al_srvs/GetCollisionObject.h"
#include "al_srvs/GetEntity.h"
#include "al_srvs/GetScene.h"
#include "al_srvs/GetEntityVisualFeatures.h"
#include "al_msgs/CollisionObjects.h"
#include "al_msgs/Entities.h"
#include "al_srvs/PerceptionAll.h"

#include "al_srvs/Perception.h"
#include "fstream"

namespace al
{
  namespace memory
  {
    const float WORKING_MEMORY_SPAN = 10.0; //10 seconds of history is stored

    class Memory
    {
    public:
      Memory (ros::NodeHandle* nh);
      virtual
      ~Memory ();

      void
      run ();

    protected:
      ros::NodeHandle* nh_;
      ros::Subscriber sub_entities_;
      ros::ServiceServer srv_col_obj_;//serves the object by id
      ros::ServiceServer srv_entity_;//serves the entity by id
      ros::ServiceServer srv_scene_;//serves the scene by its time-stamp

      ros::ServiceServer srv_perception_; //this is a temporary service
      ros::ServiceClient srv_cl_get_entity_visual_features_;
      int cnt_perception_;
      bool entity_feature_extracted_;
      std::ofstream out_file_;
      al_msgs::FeatureVector entity_features_;
      ros::Publisher pub_entity_features_;

      ros::ServiceServer srv_perception_all_;
      al_msgs::Entity entity_;

      std::vector<arm_navigation_msgs::CollisionObject> collision_objects_;
      std::map<float, al_msgs::Entities> map_memory_;

      //fetches the entity from the memory
      al_msgs::Entity*
      getEntity (std::string id, bool latest_scene_required = true);

      bool
      getChange (al_msgs::FeatureVector &change_vector, const al_msgs::FeatureVector &before_behavior,
                 const al_msgs::FeatureVector &after_behavior);

      void
      entitiesCallback (al_msgs::EntitiesConstPtr entities);

      bool
      getSceneCallback (al_srvs::GetScene::Request &req, al_srvs::GetScene::Response &res);

      bool
      getEntityCallback (al_srvs::GetEntity::Request &req, al_srvs::GetEntity::Response &res);

      bool
      getColObjCallback (al_srvs::GetCollisionObject::Request &req, al_srvs::GetCollisionObject::Response &res);

      bool
      srvPerception (al_srvs::Perception::Request &req, al_srvs::Perception::Response &res);
    };
  }
}

#endif /* MEMORY_H_ */
