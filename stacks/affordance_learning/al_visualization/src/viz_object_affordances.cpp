/*
 * viz_object_affordances.cpp
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

#include "ros/ros.h"

#include "al_msgs/Affordances.h"
#include "al_srvs/GetLearnedEffects.h"
//#include "al_srvs/GetEntity.h"
#include "al_srvs/GetEntitiesVisualFeatures.h"
#include "al_srvs/GetScene.h"

#include "al_utils/al_utils.h"

const uint TOP_N_EFFECTS = 5;

bool
getAllScene (al_msgs::Entities& entities);

ros::NodeHandle* nh_;
ros::Publisher pub_affordances_;
ros::ServiceClient srv_cl_affordances_;
al_srvs::GetScene srv_scene_;
al_srvs::GetEntitiesVisualFeatures srv_entities_features_;
al_srvs::GetLearnedEffects::Request req_learned_effects;
al_srvs::GetLearnedEffects::Response res_learned_effects;
al_msgs::Affordances affordances_;
al_msgs::Entities entities_;

std::map<float, al_msgs::Effect> map_effects;
std::map<float, al_msgs::Effect>::iterator it;

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_object_affordances");
  nh_ = new ros::NodeHandle ();
  //  ros::ServiceClient srv_cl_get_entity_;
  //  ros::ServiceClient srv_cl_get_entity_features_;

  pub_affordances_ = nh_->advertise<al_msgs::Affordances> ("/affordances", 1);

  srv_cl_affordances_ = nh_->serviceClient<al_srvs::GetLearnedEffects> ("/get_learned_effects", true);
  while (nh_->ok () && !srv_cl_affordances_.waitForExistence (ros::Duration (0.1)))
  {
    ros::spinOnce ();
  }

  //  srv_cl_get_entity_ = nh_->serviceClient<al_srvs::GetEntity> ("/memory/get_entity", true);
  //  while (nh_->ok () && !srv_cl_get_entity_.exists ())
  //  {
  //    srv_cl_get_entity_.waitForExistence (ros::Duration (0.1));
  //    ros::spinOnce ();
  //  }
  //
  //  srv_cl_get_entity_features_ = nh_->serviceClient<al_srvs::GetEntityVisualFeatures> ("/get_entity_visual_features",
  //                                                                                      true);
  //  while (nh_->ok () && !srv_cl_get_entity_features_.exists ())
  //  {
  //    srv_cl_get_entity_features_.waitForExistence (ros::Duration (0.1));
  //    ros::spinOnce ();
  //  }

  al::system::changeInputMode (1);
  getAllScene (entities_);

  //  ros::Rate r (5);
  //  std::cout << "Enter an object id to track its affordances" << std::endl;
  //  int id = -1;
  //  while (nh_->ok ())
  //  {
  //    char c_id;
  //    if (al::system::getKey (c_id))
  //    {
  //      std::cin >> c_id;
  //      id = atoi (&c_id);
  //      std::cout << "Enter an object id to track its affordances" << std::endl;
  //    }
  //
  //    if (id != -1)
  //    {
  //      //get object
  //      al_srvs::GetEntity::Request req_entity;
  //      al_srvs::GetEntity::Response res_entity;
  //      req_entity.id = al::facilities::toString (id);
  //      req_entity.latest_scene_required = true;
  //      if (srv_cl_get_entity_.call (req_entity, res_entity))
  //      {
  //        //get object features
  //        al_srvs::GetEntityVisualFeatures::Request req_features;
  //        al_srvs::GetEntityVisualFeatures::Response res_features;
  //
  //        req_features.entity = res_entity.entity;
  //        if (srv_cl_get_entity_features_.call (req_features, res_features))
  //        {
  //          al_srvs::GetLearnedEffects::Request req_learned_effects;
  //          al_srvs::GetLearnedEffects::Response res_learned_effects;
  //
  //          req_learned_effects.feature_vector = res_features.feature_vector;
  //          if (srv_cl_affordances_.call (req_learned_effects, res_learned_effects))
  //          {
  //            for (uint i = 0; i < res_learned_effects.effects.size (); i++)
  //            {
  //              std::cout << al::facilities::behaviorEnumToString (res_learned_effects.effects[i].arg) << "\t";
  //              std::cout << al::facilities::effectEnumToString (res_learned_effects.effects[i].effect) << "\t";
  //              std::cout << res_learned_effects.effects[i].prob << std::endl;
  //              //              std::cout << res_learned_effects.effects[i] << std::endl;
  //              map_effects[res_learned_effects.effects[i].prob] = res_learned_effects.effects[i];
  //            }
  //
  //            //now pick top 5 different affordances
  //            //convert into an affordance message and publish
  //            it = map_effects.end ();
  //            //            affordances_.effects.resize (TOP_N_EFFECTS);
  //            while (it != map_effects.begin () && affordances_.effects.size () < TOP_N_EFFECTS)
  //            {
  //              --it;
  //              bool same_effect_exists = false;
  //              for (uint i = 0; i < affordances_.effects.size (); i++)
  //              {
  //                if (it->second.effect == affordances_.effects[i].effect)
  //                {
  //                  same_effect_exists = true;
  //                  if (it->first > affordances_.effects[i].prob)
  //                  {
  //                    affordances_.effects[i].effect = it->second.effect;
  //                    break;
  //                  }
  //                }
  //              }
  //              if (!same_effect_exists)
  //              {
  //                affordances_.effects.push_back (it->second);
  //              }
  //            }
  //
  //            affordances_.object = res_entity.entity.collision_object;
  //            std::cout << "n_affordances: " << (int)affordances_.effects.size () << std::endl;
  //            pub_affordances_.publish (affordances_);
  //            map_effects.clear ();
  //            affordances_.effects.clear ();
  //          }
  //        }
  //      }
  //    }
  //    ros::spinOnce ();
  //    r.sleep ();
  //  }

  al::system::changeInputMode (0);
  return 0;
}

bool
getAllScene (al_msgs::Entities& entities)
{
  ros::Rate r (10);
  while (nh_->ok ())
  {
    if (pub_affordances_.getNumSubscribers ())
    {
      //visualize affordances if anyone cares
      srv_scene_.request.seconds_ago = 10;
      srv_scene_.request.header.stamp = ros::Time::now ();
      bool success = ros::service::call ("/get_scene", srv_scene_.request, srv_scene_.response);
      if (success)
      {
        entities = srv_scene_.response.scene.entities;
        uint n_objects = entities.entities.size ();
        //        std::cout << "n_objects: " << n_objects << std::endl;
        //        for (uint i = 0; i < n_objects; i++)
        //        {
        //          std::cout << "object   :" << srv_scene_.response.scene.entities.entities[i].collision_object.id << "\n";
        //          std::cout << "box_x    :"
        //              << srv_scene_.response.scene.entities.entities[i].collision_object.poses[0].position.x << "\n";
        //          std::cout << "box_y    :"
        //              << srv_scene_.response.scene.entities.entities[i].collision_object.poses[0].position.y << "\n";
        //          std::cout << "box_z    :"
        //              << srv_scene_.response.scene.entities.entities[i].collision_object.poses[0].position.z << "\n";
        //        }

        srv_entities_features_.request.entities = entities;
        success = ros::service::call ("/get_entities_visual_features", srv_entities_features_.request,
                                      srv_entities_features_.response);
        if (success)
        {
          uint n_objects = srv_entities_features_.request.entities.entities.size ();
          for (uint i = 0; i < n_objects; i++)
          {
            if (!al::facilities::toInt (entities.entities[i].collision_object.id))
              continue;

            entities.entities[i].feature_vector = srv_entities_features_.response.feature_vectors[i];
            //            std::cout << "entity: " << srv_entities_features_.request.entities.entities[i].collision_object.id
            //                << "\t #features: " << (int)srv_entities_features_.response.feature_vectors[i].features.size ()
            //                << std::endl;


            req_learned_effects.feature_vector = entities.entities[i].feature_vector;
            if (srv_cl_affordances_.call (req_learned_effects, res_learned_effects))
            {
              for (uint j = 0; j < res_learned_effects.effects.size (); j++)
              {
                std::cout << al::facilities::behaviorEnumToString (res_learned_effects.effects[j].arg) << "\t";
                std::cout << al::facilities::effectEnumToString (res_learned_effects.effects[j].effect) << "\t";
                std::cout << res_learned_effects.effects[j].prob << std::endl;
                //              std::cout << res_learned_effects.effects[i] << std::endl;
                map_effects[res_learned_effects.effects[j].prob] = res_learned_effects.effects[j];
              }

              //now pick top 5 different affordances
              //convert into an affordance message and publish
              it = map_effects.end ();
              while (it != map_effects.begin () && affordances_.effects.size () < TOP_N_EFFECTS)
              {
                --it;
                bool same_effect_exists = false;
                for (uint j = 0; j < affordances_.effects.size (); j++)
                {
                  if (it->second.effect == affordances_.effects[j].effect)
                  {
                    same_effect_exists = true;
                    if (it->first > affordances_.effects[j].prob)
                    {
                      affordances_.effects[j].effect = it->second.effect;
                      break;
                    }
                  }
                }
                if (!same_effect_exists)
                {
                  affordances_.effects.push_back (it->second);
                }
              }
              affordances_.object = entities.entities[i].collision_object;
              std::cout << "n_affordances: " << (int)affordances_.effects.size () << std::endl;
              pub_affordances_.publish (affordances_);
              map_effects.clear ();
              affordances_.effects.clear ();
            }
            else
            {
              std::cout << "error while getting learned affordances of the object" << std::endl;
            }
          }
        }
        else
        {
          std::cout << "problem with feature extraction" << std::endl;
        }
        //      std::cout << "box_size :" << srv_scene_.response.scene.entities.entities[i].feature_vector.features.size ()
        //          << "\n";
      }
      else
        std::cout << "problem with scene extraction" << std::endl;
    }
    ros::spinOnce ();
    r.sleep ();
  }
}
