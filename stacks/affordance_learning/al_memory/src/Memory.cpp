/*
 * Memory.cpp
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

#include "al_memory/Memory.h"

namespace al
{
  namespace memory
  {
    Memory::Memory (ros::NodeHandle* nh) :
        nh_ (nh)
    {
      std::cout << "1" << std::endl;
      sub_entities_ = nh_->subscribe ("/perception/entities", 1, &Memory::entitiesCallback, this);
      srv_col_obj_ = nh_->advertiseService ("/memory/get_collision_object", &Memory::getColObjCallback, this);
      srv_entity_ = nh_->advertiseService ("/memory/get_entity", &Memory::getEntityCallback, this);

      srv_perception_ = nh_->advertiseService ("/perception", &Memory::srvPerception, this);
      srv_scene_ = nh_->advertiseService ("/get_scene", &Memory::getSceneCallback, this);
      std::cout << "1*" << std::endl;
      srv_cl_get_entity_visual_features_ = nh_->serviceClient<al_srvs::GetEntityVisualFeatures> (
          "/get_entity_visual_features", true);
      srv_cl_get_entity_visual_features_.waitForExistence ();
      std::cout << "1**" << std::endl;

      pub_entity_features_ = nh_->advertise<al_msgs::FeatureVector> ("/feature_vector", 1);

      cnt_perception_ = 0;
      entity_feature_extracted_ = false;
    }

    Memory::~Memory ()
    {
      // TODO Auto-generated destructor stub
    }

    al_msgs::Entity*
    Memory::getEntity (std::string id, bool latest_scene_required)
    {
      al_msgs::Entity* entity = NULL;

      std::map<float, al_msgs::Entities>::iterator it = map_memory_.end ();
      //      if (map_memory_.size ())
      //      {
      //        for (; it != map_memory_.begin (); --it)
      //          for (uint j = 0; j < it->second.entities.size (); j++)
      //            std::cout << "time_acquired: " << it->first << "\t" << it->second.entities[j].cloud_object.height
      //                * it->second.entities[j].cloud_object.width << "\n";
      //
      //        for (uint j = 0; j < map_memory_.begin ()->second.entities.size (); j++)
      //          std::cout << map_memory_.begin ()->first << "\t"
      //              << map_memory_.begin ()->second.entities[j].cloud_object.height
      //                  * map_memory_.begin ()->second.entities[j].cloud_object.width << "\n";
      //      }
      //
      //      it = map_memory_.end ();
      if (!map_memory_.size ())
        return NULL;

      std::cout << "a" << std::endl;
      if (latest_scene_required)
      {
        std::cout << "b" << std::endl;
        if (it != map_memory_.begin ())
        {
          std::cout << "c" << std::endl;
          --it;
        }
        else
        {
          std::cout << "d" << std::endl;
          return NULL;
        }
        for (uint i = 0; i < it->second.entities.size (); i++)
        {
          std::cout << "e" << std::endl;
          if (it->second.entities[i].collision_object.id == id)
          {
            std::cout << "f" << std::endl;
            entity_ = it->second.entities[i];
            entity = &entity_;
            //            entity = &it->second.entities[i];
            ROS_WARN("an object fetched from working memory! w/ latest scene required");
            ROS_WARN("Now the time is: %f", al::math::fPrecision (ros::Time::now ()));
            ROS_WARN("Object is from : %f", it->first);
            return entity;
          }
        }
      }
      else //check the memory if this object is seen before, and return this
      {
        std::cout << "g" << std::endl;
        while (it != map_memory_.begin ())
        {
          std::cout << "h" << std::endl;
          --it;
          for (uint i = 0; i < it->second.entities.size (); i++)
          {
            std::cout << "i" << std::endl;
            if (it->second.entities[i].collision_object.id == id)
            {
              std::cout << "j" << std::endl;
              entity_ = it->second.entities[i];
              entity = &entity_;
              ROS_WARN("an object fetched from working memory!");
              ROS_WARN("Now the time is: %f", al::math::fPrecision (ros::Time::now ()));
              ROS_WARN("Object is from : %f", it->first);
              return entity;
            }
          }
        }
      }
      if (latest_scene_required)
        ROS_WARN("object %s requested, but there is no such object in the latest scene", (char*)id.c_str());
      else
        ROS_WARN(
            "object %s requested, but there is no such object seen in the last %f seconds", (char*)id.c_str(), WORKING_MEMORY_SPAN);
      return NULL;
    }

    bool
    Memory::getSceneCallback (al_srvs::GetScene::Request &req, al_srvs::GetScene::Response &res)
    {
      float t_req_scene = al::math::fPrecision (req.header.stamp);
      float seconds_ago = req.seconds_ago;
      map_memory_[t_req_scene] = res.scene.entities;
      std::map<float, al_msgs::Entities>::iterator it;
      it = map_memory_.find (t_req_scene);
      std::cout << "1" << std::endl;
      if (it == map_memory_.begin ())
      {
        std::cout << "2" << std::endl;
        //memory is empty or requested time is too old compared to the WORKING_MEMORY_SPAN
        ROS_WARN("memory is empty or requested time is too old compared to the WORKING_MEMORY_SPAN");
        map_memory_.erase (map_memory_.begin ());
        return false;
      }

      while (it != map_memory_.begin ())
      {
        float t_prev_scene = (--it)->first;
        if (t_prev_scene > t_req_scene - seconds_ago)
        {
          std::cout << "3" << std::endl;
          res.scene.entities = it->second;
          map_memory_.erase (t_req_scene);
          return true;
        }
      }
      ROS_WARN(
          "there is no scene data corresponding to the time %f or before %f seconds than that.", t_req_scene, seconds_ago);
      map_memory_.erase (t_req_scene);
      std::cout << "4" << std::endl;
      return false;
    }

    bool
    Memory::getEntityCallback (al_srvs::GetEntity::Request &req, al_srvs::GetEntity::Response &res)
    {
      al_msgs::Entity* entity = getEntity (req.id);
      if (entity != NULL)
      {
        res.entity = *entity;
        return true;
      }
      else
      {

        return false;
      }
    }

    void
    Memory::entitiesCallback (al_msgs::EntitiesConstPtr entities)
    {
      //      std::cout << entities->header.stamp << std::endl;
      float t_scene_acquisition = al::math::fPrecision (entities->header.stamp, 4, 3);
      //      std::cout << "time: " << t_scene_acquisition << std::endl;

      std::map<float, al_msgs::Entities>::iterator it = map_memory_.begin ();
      while (it != map_memory_.end ())
      {
        if (al::math::fPrecision (ros::Time::now (), 4, 3) - it->first > WORKING_MEMORY_SPAN)
          map_memory_.erase (it++);
        else
          break;
      }

      map_memory_[t_scene_acquisition] = *entities;
      //      std::cout << "#scene memorized: " << (int)map_memory_.size () << std::endl;
      //      std::cout << "t_first_scene :" << map_memory_.begin ()->first << std::endl;
      it = map_memory_.end ();
      //      std::cout << "t_last__scene :" << (--it)->first << std::endl;
    }

    bool
    Memory::getColObjCallback (al_srvs::GetCollisionObject::Request &req, al_srvs::GetCollisionObject::Response &res)
    {
      std::cout << "collision object request done with the id " << req.id << std::endl;
      al_msgs::Entity* entity = getEntity (req.id);
      if (entity != NULL)
      {
        res.collision_object = entity->collision_object;
        return true;
      }
      else
        return false;

      /*
       std::map<float, al_msgs::Entities>::iterator it = map_memory_.end ();
       if (req.latest_scene_required)
       {
       --it;
       for (uint i = 0; i < it->second.entities.size (); i++)
       {
       if (it->second.entities[i].collision_object.id == req.id)
       {
       res.collision_object = it->second.entities[i].collision_object;
       ROS_WARN("an object fetched from working memory! w/ latest scene required");
       ROS_WARN("Now the time is: %f",al::math::fPrecision (ros::Time::now ()));
       ROS_WARN("Object is from : %f",it->first);
       return true;
       }
       }
       }
       else //check the memory if this object is seen before, and return this
       {
       while (it != map_memory_.begin ())
       {
       --it;
       for (uint i = 0; i < it->second.entities.size (); i++)
       {
       if (it->second.entities[i].collision_object.id == req.id)
       {
       res.collision_object = it->second.entities[i].collision_object;
       ROS_WARN("an object fetched from working memory!");
       ROS_WARN("Now the time is: %f",al::math::fPrecision (ros::Time::now ()));
       ROS_WARN("Object is from : %f",it->first);
       return true;
       }
       }
       }
       }
       if (req.latest_scene_required)
       ROS_WARN("object %s requested, but there is no such object in the latest scene", (char*)req.id.c_str());
       else
       ROS_WARN("object %s requested, but there is no such object seen in the last %f seconds", (char*)req.id.c_str(), WORKING_MEMORY_SPAN);
       return false;
       */
    }

    void
    Memory::run ()
    {
      ros::Rate r (30);
      while (nh_->ok ())
      {
        //        al_msgs::Entity* entity = NULL;
        //        entity = getEntity (al::facilities::toString (1), true);
        //        if (entity != NULL)
        //        {
        //          al_srvs::GetEntityVisualFeatures::Request req_features;
        //          al_srvs::GetEntityVisualFeatures::Response res_features;
        //          req_features.entity = *entity;
        //          if (!srv_cl_get_entity_visual_features_.call (req_features, res_features))
        //          {
        //            //TODO: this can be handled better
        //            ROS_WARN("entity feature calculation service call failed!");
        //            return;
        //          }
        //          else
        //          {
        //            std::cout<<res_features.feature_vector<<std::endl;
        //          }
        //        }

        ros::spinOnce ();
        r.sleep ();
      }
    }

    bool
    Memory::getChange (al_msgs::FeatureVector &change_vector, const al_msgs::FeatureVector &before_behavior,
                       const al_msgs::FeatureVector &after_behavior)
    {
      if (before_behavior.features.size () != after_behavior.features.size ())
      {
        ROS_WARN("wrong number of features, one of the vectors should be padded with default features!");
        return false;
      }

      for (uint i = 0; i < before_behavior.features.size (); i++)
      {
        if (before_behavior.features[i].his.size () != after_behavior.features[i].his.size ())
        {
          ROS_WARN(
              "two vectors have different sized histogram features, probably different-typed features are subtracted!");
          return false;
        }
      }

      change_vector.features.resize (before_behavior.features.size ());
      for (uint i = 0; i < before_behavior.features.size (); i++)
      {
        change_vector.features[i].min = after_behavior.features[i].min - before_behavior.features[i].min;
        change_vector.features[i].max = after_behavior.features[i].max - before_behavior.features[i].max;
        change_vector.features[i].avg = after_behavior.features[i].avg - before_behavior.features[i].avg;
        change_vector.features[i].var = after_behavior.features[i].var - before_behavior.features[i].var;
        change_vector.features[i].dev = after_behavior.features[i].dev - before_behavior.features[i].dev;
        change_vector.features[i].type = before_behavior.features[i].type;
        change_vector.features[i].his.resize (before_behavior.features[i].his.size ());
        for (uint j = 0; j < change_vector.features[i].his.size (); j++)
          change_vector.features[i].his[j] = after_behavior.features[i].his[j] - before_behavior.features[i].his[j];
      }
      return true;
    }

    bool
    Memory::srvPerception (al_srvs::Perception::Request &req, al_srvs::Perception::Response &res)
    {
      res.pushable_object_center.resize (3);
      res.pushable_object_size.resize (3);

      std::cout << "TASK IS:" << (int)req.task << std::endl;

      if (req.task == al_srvs::Perception::Request::DO_PERCEPT)
      {
        std::cout << "DO_PERCEPT REQUESTED" << std::endl;
        al_msgs::Entity* entity;
        entity = getEntity (al::facilities::toString ((int)req.arg), true);
        if (entity != NULL)
        {
          res.pushable_object_center[0] = entity->collision_object.poses[0].position.x;
          res.pushable_object_center[1] = entity->collision_object.poses[0].position.y;
          res.pushable_object_center[2] = entity->collision_object.poses[0].position.z;

          tf::Quaternion tf_q;
          tf::quaternionMsgToTF (entity->collision_object.poses[0].orientation, tf_q);
          res.pushable_object_yaw = tf::getYaw (tf_q);

          res.pushable_object_size[0] = entity->collision_object.shapes[0].dimensions[0];
          res.pushable_object_size[1] = entity->collision_object.shapes[0].dimensions[1];
          res.pushable_object_size[2] = entity->collision_object.shapes[0].dimensions[2];

          std::cout << "*****************************************************************" << std::endl;
          std::cout << res.pushable_object_center[0] << " " << res.pushable_object_center[1] << " "
              << res.pushable_object_center[2] << std::endl;

          std::cout << res.pushable_object_size[0] << " " << res.pushable_object_size[1] << " "
              << res.pushable_object_size[2] << std::endl;
          std::cout << "*****************************************************************" << std::endl;

          al_srvs::GetEntityVisualFeatures::Request req_features;
          al_srvs::GetEntityVisualFeatures::Response res_features;
          req_features.entity = *entity;
          std::cout << "id   : " << req_features.entity.collision_object.id << std::endl;
          std::cout << "n_pts: " << req_features.entity.cloud_object.width << std::endl;
          std::cout << "pos  : " << req_features.entity.collision_object.poses[0].position.x << " "
              << req_features.entity.collision_object.poses[0].position.y << " "
              << req_features.entity.collision_object.poses[0].position.z << " " << std::endl;
          //          std::cout << req_features.entity.cloud_object.width * req_features.entity.cloud_object.height << std::endl;
          //          std::cout << req_features.entity.feature_vector.features.size () << std::endl;
          if (!srv_cl_get_entity_visual_features_.call (req_features, res_features))
          {
            ROS_WARN("entity feature calculation service call failed!");
            //            std::cout << "failed: " << (int)res_features.feature_vector.features.size () << std::endl;
            //            std::cout << res_features.feature_vector << std::endl;
            //TODO: this can be handled better
            return false;
          }

          //          std::cout << "nFeatures: " << (int)res_features.feature_vector.features.size () << std::endl;
          //          std::cout << res_features.feature_vector << std::endl;
          if (!res_features.feature_vector.features.size ())
          {
            ROS_WARN("entity has no features!");
            return false;
          }
          entity_feature_extracted_ = true;
          pub_entity_features_.publish (res_features.feature_vector);
          entity_features_ = res_features.feature_vector;
          //          al_msgs::FeatureVector feature_vector = res_features.feature_vector;
          //          entity_features_ = feature_vector;
          //          //TODO: save/log this feature_vector
          //          out_file_.open (("entity_" + al::facilities::toString (cnt_perception_)).c_str ());
          //          out_file_ << feature_vector << std::endl;
          //          out_file_.close ();
          return true;
        }
        else
        {
          //we cannot extract feature in this case
          ROS_WARN("no such object in the scene!");

          //creating virtual object
          res.pushable_object_center[0] = -0.3;
          res.pushable_object_center[1] = -0.2;
          res.pushable_object_center[2] = 0.0;

          res.pushable_object_size[0] = 0.06;
          res.pushable_object_size[1] = 0.06;
          res.pushable_object_size[2] = 0.06;
          //          return false;
          return true;
        }
      }
      else if (req.task == al_srvs::Perception::Request::EXTRACT_EFFECT)
      {
        std::cout << "EXTRACT EFFECT REQUESTED" << std::endl;
        std::cout << "entity feature vector size: " << (int)entity_features_.features.size () << std::endl;

        al_msgs::Entity* entity = NULL;
        entity = getEntity (al::facilities::toString ((int)req.arg), true);
        if (entity != NULL)
        {
          std::cout << "haydeee" << std::endl;
          res.pushable_object_center[0] = entity->collision_object.poses[0].position.x;
          res.pushable_object_center[1] = entity->collision_object.poses[0].position.y;
          res.pushable_object_center[2] = entity->collision_object.poses[0].position.z;

          res.pushable_object_size[0] = entity->collision_object.shapes[0].dimensions[0];
          res.pushable_object_size[1] = entity->collision_object.shapes[0].dimensions[1];
          res.pushable_object_size[2] = entity->collision_object.shapes[0].dimensions[2];

          std::cout << "*****************************************************************" << std::endl;
          std::cout << res.pushable_object_center[0] << " " << res.pushable_object_center[1] << " "
              << res.pushable_object_center[2] << std::endl;

          std::cout << res.pushable_object_size[0] << " " << res.pushable_object_size[1] << " "
              << res.pushable_object_size[2] << std::endl;
          std::cout << "*****************************************************************" << std::endl;

          al_srvs::GetEntityVisualFeatures::Request req_features;
          al_srvs::GetEntityVisualFeatures::Response res_features;
          req_features.entity = *entity;
          if (!srv_cl_get_entity_visual_features_.call (req_features, res_features))
          {
            //TODO: this can be handled better
            ROS_WARN("entity feature calculation service call failed!");
            return false;
          }

          if (!res_features.feature_vector.features.size ())
          {
            ROS_WARN("entity has no features!");
            return false;
          }
          al_msgs::FeatureVector feature_vector = res_features.feature_vector;
          al_msgs::FeatureVector effect_features_;
          std::cout << entity_features_.features.size () << " " << feature_vector.features.size () << std::endl;
          if (getChange (effect_features_, entity_features_, feature_vector))
          {
            pub_entity_features_.publish (effect_features_);
            //            out_file_.open (("effect_" + al::facilities::toString (cnt_perception_)).c_str ());
            //            out_file_ << effect_features_ << std::endl;
            //            out_file_.close ();
            entity_feature_extracted_ = false;
            cnt_perception_++;
            return true;
          }
          else
            return false;
        }
        else
        {
          //creating virtual object
          /*
           res.pushable_object_center[0] = 0.0;
           res.pushable_object_center[1] = 0.0;
           res.pushable_object_center[2] = 0.0;

           res.pushable_object_size[0] = 0.0;
           res.pushable_object_size[1] = 0.0;
           res.pushable_object_size[2] = 0.0;
           */
          //therefore object is disappeared
          //hence the change is almost the negation of entity vector
          al_msgs::FeatureVector feature_vector;
          std::cout << "h" << std::endl;
          //          feature_vector.features.resize (entity_features_.features.size ());
          feature_vector = entity_features_;
          for (uint i = 0; i < feature_vector.features.size (); i++)
          {
            //            std::cout<<"h"<<std::endl;
            feature_vector.features[i].min = 0.0;
            feature_vector.features[i].max = 0.0;
            feature_vector.features[i].avg = 0.0;
            feature_vector.features[i].dev = 0.0;
            feature_vector.features[i].var = 0.0;
            //            feature_vector.features[i].his.resize (entity_features_.features[i].his.size ());
            std::cout << "hist size of feature: " << (int)i << " is " << (int)feature_vector.features[i].his.size ()
                << std::endl;
            for (uint j = 0; j < feature_vector.features[i].his.size (); j++)
              feature_vector.features[i].his[j] = 0.0;
          }

          al_msgs::FeatureVector effect_features_;
          if (getChange (effect_features_, entity_features_, feature_vector))
          {
            pub_entity_features_.publish (effect_features_);
            //            out_file_.open (("effect_" + al::facilities::toString (cnt_perception_)).c_str ());
            //            out_file_ << effect_features_ << std::endl;
            //            out_file_.close ();
            entity_feature_extracted_ = false;
            cnt_perception_++;
            std::cout << "effect feature calculated" << std::endl;
            return true;
          }
          else
          {
            std::cout << "effect feature couldn't be calculated" << std::endl;
            return false;
          }
        }
      }
      return false;
    }
  }
}
