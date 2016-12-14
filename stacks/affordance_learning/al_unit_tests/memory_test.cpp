/*
 * memory_test.cpp
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

#include "al_srvs/GetEntitiesVisualFeatures.h"
#include "al_srvs/Perception.h"
#include "al_srvs/GetScene.h"
#include "ros/ros.h"

#include "string"

ros::NodeHandle* nh_;
al_srvs::Perception srv_perception;
al_srvs::GetScene srv_scene_;
al_srvs::GetEntitiesVisualFeatures srv_entities_features_;

void
randEntityFeatureGetter ();

void
getSceneAllTime ();

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_unit_tests");
  nh_ = new ros::NodeHandle ();

  getSceneAllTime ();

  //  ros::Rate r (50);
  //  bool entered = false;
  //  bool success;
  //  while (nh_->ok ())
  //  {
  //    if (!entered)
  //    {
  //      std::cout << "Press an entity id to request its features!" << std::endl;
  //      int id;
  //      std::cin >> id;
  //      entered = true;
  //      srv_perception.request.task = al_srvs::Perception::Request::DO_PERCEPT;
  //      srv_perception.request.arg = (int8_t)id;
  //      success = ros::service::call ("/perception", srv_perception.request, srv_perception.response);
  //      entered = false;
  //      if (success)
  //      {
  //        std::cout << "service call successful" << std::endl;
  //        std::cout << srv_perception.response.pushable_object_center[0] << " "
  //            << srv_perception.response.pushable_object_center[1] << " "
  //            << srv_perception.response.pushable_object_center[2] << "\n";
  //      }
  //      else
  //      {
  //        std::cout << "service call failed" << std::endl;
  //      }
  //    }
  //    ros::spinOnce ();
  //    r.sleep ();
  //  }

  return 0;
}

void
randEntityFeatureGetter ()
{

}

void
getSceneAllTime ()
{
  ros::Rate r (10);
  while (nh_->ok ())
  {
    srv_scene_.request.seconds_ago = 10;
    srv_scene_.request.header.stamp = ros::Time::now ();
    bool success = ros::service::call ("/get_scene", srv_scene_.request, srv_scene_.response);
    if (success)
    {
      uint n_objects = srv_scene_.response.scene.entities.entities.size ();
      std::cout << "n_objects: " << n_objects << std::endl;
      for (uint i = 0; i < n_objects; i++)
      {
        //        std::cout << "object   :" << srv_scene_.response.scene.entities.entities[i].collision_object.id << "\n";
        //        std::cout << "box_x    :"
        //            << srv_scene_.response.scene.entities.entities[i].collision_object.poses[0].position.x << "\n";
        //        std::cout << "box_y    :"
        //            << srv_scene_.response.scene.entities.entities[i].collision_object.poses[0].position.y << "\n";
        //        std::cout << "box_z    :"
        //            << srv_scene_.response.scene.entities.entities[i].collision_object.poses[0].position.z << "\n";
      }

      srv_entities_features_.request.entities = srv_scene_.response.scene.entities;
      success = ros::service::call ("/get_entities_visual_features", srv_entities_features_.request,
                                    srv_entities_features_.response);
      if (success)
      {
        uint n_objects = srv_entities_features_.request.entities.entities.size ();
        for (uint i = 0; i < n_objects; i++)
        {
          //          std::cout << "entity: " << srv_entities_features_.request.entities.entities[i].collision_object.id
          //              << "\t #features: " << (int)srv_entities_features_.response.feature_vectors[i].features.size () << "\n";
          for (uint j = 0; j < srv_entities_features_.response.feature_vectors[i].features.size (); j++)
            if (srv_entities_features_.response.feature_vectors[i].features[j].type == al_msgs::Feature::HUMAN_PRESENCE)
            {
              std::cout << "presence: " << srv_entities_features_.response.feature_vectors[i].features[j].avg
                  << std::endl;
              std::cout << "torso z : " << srv_entities_features_.response.feature_vectors[i].features[j - 7].avg
                  << std::endl;
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

    ros::spinOnce ();
    r.sleep ();
  }
}
