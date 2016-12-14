/*
 * viz_scene_affordances.cpp
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

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#include "al_msgs/Shapes.h"
#include "al_msgs/SceneAffordances.h"
#include "al_msgs/SceneObjects.h"
#include "al_srvs/GetLearnedEffects.h"
#include "al_srvs/GetLearnedObjectShapes.h"
#include "al_srvs/GetEntitiesVisualFeatures.h"
#include "al_srvs/GetScene.h"
#include "al_utils/al_utils.h"

const uint TOP_N_EFFECTS = 5;

bool
getSceneFeatures (al_msgs::Entities& entities);

bool
getSceneAffordances (const al_msgs::Entities& entities, al_msgs::SceneAffordances& scene_affordances);

bool
getSceneObjects (const al_msgs::Entities& entities, al_msgs::SceneObjects& scene_objects);

bool
vizSceneAffordances (const al_msgs::SceneAffordances& scene_affordances);

bool
vizSceneObjects (const al_msgs::SceneObjects& scene_objects);

ros::NodeHandle* nh_;
//ros::Publisher pub_affordances_;
ros::Publisher pub_affordance_markers_;
ros::Publisher pub_shape_markers_;
ros::ServiceClient srv_cl_affordances_;
ros::ServiceClient srv_cl_categories_;
al_srvs::GetScene srv_scene_;
al_srvs::GetEntitiesVisualFeatures srv_entities_features_;
al_srvs::GetLearnedEffects::Request req_learned_effects;
al_srvs::GetLearnedEffects::Response res_learned_effects;
al_srvs::GetLearnedObjectShapes::Request req_learned_shapes;
al_srvs::GetLearnedObjectShapes::Response res_learned_shapes;
al_msgs::Entities entities_;

visualization_msgs::MarkerArray aff_labels_;
visualization_msgs::MarkerArray prev_aff_labels_;

visualization_msgs::MarkerArray shape_labels_;
visualization_msgs::MarkerArray prev_shape_labels_;

std::map<float, al_msgs::Effect> map_effects;
std::map<float, al_msgs::Effect>::iterator it;

std::map<float, al_msgs::Shape> map_shapes;
std::map<float, al_msgs::Shape>::iterator it_shape;

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_object_affordances");
  nh_ = new ros::NodeHandle ();

  //  pub_affordances_ = nh_->advertise<al_msgs::Affordances> ("/affordances", 1);
  pub_affordance_markers_ = nh_->advertise<visualization_msgs::MarkerArray> ("affordance_labels", 1);
  pub_shape_markers_ = nh_->advertise<visualization_msgs::MarkerArray> ("shape_labels", 1);

  srv_cl_affordances_ = nh_->serviceClient<al_srvs::GetLearnedEffects> ("/get_learned_effects", true);
  srv_cl_categories_ = nh_->serviceClient<al_srvs::GetLearnedObjectShapes> ("/get_learned_object_shapes", true);
  std::cout << "waiting for -get_learned_effects- service to come up" << std::endl;
  while (nh_->ok () && !srv_cl_affordances_.waitForExistence (ros::Duration (0.1)))
    ros::spinOnce ();
  std::cout << "connected to -get_learned_effects- service!" << std::endl;

  ros::Rate r (10);
  while (nh_->ok ())
  {
    bool got_scene = false;
    if (pub_affordance_markers_.getNumSubscribers ())
    {
      if (getSceneFeatures (entities_))
      {
        got_scene = true;
        al_msgs::SceneAffordances scene_affordances;
        if (getSceneAffordances (entities_, scene_affordances))
        {
          //now visualize affordances
          vizSceneAffordances (scene_affordances);
        }
      }
    }

    if (pub_shape_markers_.getNumSubscribers ())
    {
      if (!got_scene)
      {
        if (getSceneFeatures (entities_))
        {
          got_scene = true;
        }
      }

      if (got_scene)
      {
        al_msgs::SceneObjects scene_objects;
        getSceneObjects (entities_, scene_objects);
        if (getSceneObjects (entities_, scene_objects))
          vizSceneObjects (scene_objects);
      }
    }
    ros::spinOnce ();
    r.sleep ();
  }
  return 0;
}

bool
vizSceneObjects (const al_msgs::SceneObjects& scene_objects)
{
  uint n_objects = scene_objects.objects.size ();
  uint cnt_marker_id = 0;
  shape_labels_.markers.clear ();//bad usage, use resize at the very first, than assign with cnt_marker_id
  for (uint i = 0; i < n_objects; i++)
  {
    al_msgs::Object object = scene_objects.objects[i];
    if (!al::facilities::toInt (object.object.id) || !object.shapes.size ())
      continue;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "shape_label_texts";
    marker.id = cnt_marker_id;
    cnt_marker_id++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration (5);

    geometry_msgs::Point start;
    start.x = object.object.poses[0].position.x;
    start.y = object.object.poses[0].position.y;
    start.z = object.object.poses[0].position.z;

    geometry_msgs::Point end;
    end.x = object.object.poses[0].position.x;
    if (object.object.poses[0].position.y > 0)
      end.y = 0.70;
    else
      end.y = -0.70;
    end.z = start.z - 0.30;

    marker.points.push_back (start);
    marker.points.push_back (end);

    marker.scale.x = 0.02;
    marker.scale.y = 0.04;
    //  marker.scale.z = 0.1;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8;

    shape_labels_.markers.push_back (marker);
    geometry_msgs::Point arrow_end = marker.points[1];

    for (uint8_t i = 0; i < object.shapes.size (); i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/base_footprint";
      marker.header.stamp = ros::Time::now ();
      marker.ns = "shape_label_texts";
      marker.id = cnt_marker_id;
      cnt_marker_id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = arrow_end.x;
      marker.pose.position.y = arrow_end.y;
      marker.pose.position.z = arrow_end.z - 0.09 * (i + 1);

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      std::stringstream s;
      s << (int)(object.shapes[i].prob * 100);

      std::string text = al::facilities::shapeEnumToString (object.shapes[i].shape) + "\t" + "( " + s.str () + "% )";

      marker.text = text;
      marker.scale.x = 0.15;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      shape_labels_.markers.push_back (marker);
    }
  }

  visualization_msgs::MarkerArray labels;
  labels.markers = shape_labels_.markers;

  std::vector<bool> labels_used (prev_shape_labels_.markers.size (), false);

  for (uint i = 0; i < prev_shape_labels_.markers.size (); i++)
  {
    for (uint j = 0; j < shape_labels_.markers.size (); j++)
    {
      if (prev_shape_labels_.markers[i].id == shape_labels_.markers[j].id)
      {
        labels_used[i] = true;
        break;
      }
    }
  }
  for (uint i = 0; i < labels_used.size (); i++)
  {
    if (!labels_used[i])
    {
      prev_shape_labels_.markers[i].action = visualization_msgs::Marker::DELETE;
      prev_shape_labels_.markers[i].header.stamp = ros::Time::now ();
      labels.markers.push_back (prev_shape_labels_.markers[i]);
    }
  }

  prev_shape_labels_ = shape_labels_;

  if (pub_shape_markers_.getNumSubscribers ())
    pub_shape_markers_.publish (labels);
  return true;
}

bool
vizSceneAffordances (const al_msgs::SceneAffordances& scene_affordances)
{
  uint n_objects = scene_affordances.scene_affordances.size ();
  uint cnt_marker_id = 0;
  aff_labels_.markers.clear ();//bad usage, use resize at the very first, than assign with cnt_marker_id
  for (uint i = 0; i < n_objects; i++)
  {
    al_msgs::Affordances affordances = scene_affordances.scene_affordances[i];
    if (!al::facilities::toInt (affordances.object.id) || !affordances.effects.size ())
      continue;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "aff_label_texts";
    marker.id = cnt_marker_id;
    cnt_marker_id++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration (5);

    geometry_msgs::Point start;
    start.x = affordances.object.poses[0].position.x;
    start.y = affordances.object.poses[0].position.y;
    start.z = affordances.object.poses[0].position.z;

    geometry_msgs::Point end;
    end.x = affordances.object.poses[0].position.x;
    if (affordances.object.poses[0].position.y > 0)
      end.y = 0.70;
    else
      end.y = -0.70;
    end.z = start.z + 0.30;

    marker.points.push_back (start);
    marker.points.push_back (end);

    marker.scale.x = 0.02;
    marker.scale.y = 0.04;
    //  marker.scale.z = 0.1;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8;

    aff_labels_.markers.push_back (marker);
    geometry_msgs::Point arrow_end = marker.points[1];

    for (uint8_t i = 0; i < affordances.effects.size (); i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/base_footprint";
      marker.header.stamp = ros::Time::now ();
      marker.ns = "aff_label_texts";
      marker.id = cnt_marker_id;
      cnt_marker_id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = arrow_end.x;
      marker.pose.position.y = arrow_end.y;
      marker.pose.position.z = arrow_end.z + 0.09 * (i + 1);

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      std::stringstream s;
      s << (int)(affordances.effects[i].prob * 100);

      std::string text = al::facilities::effectEnumToString (affordances.effects[i].effect) + "\t" + "( " + s.str ()
          + "% )";

      marker.text = text;
      marker.scale.x = 0.15;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      aff_labels_.markers.push_back (marker);
    }
  }

  visualization_msgs::MarkerArray labels;
  labels.markers = aff_labels_.markers;

  std::vector<bool> labels_used (prev_aff_labels_.markers.size (), false);

  for (uint i = 0; i < prev_aff_labels_.markers.size (); i++)
  {
    for (uint j = 0; j < aff_labels_.markers.size (); j++)
    {
      if (prev_aff_labels_.markers[i].id == aff_labels_.markers[j].id)
      {
        labels_used[i] = true;
        break;
      }
    }
  }
  for (uint i = 0; i < labels_used.size (); i++)
  {
    if (!labels_used[i])
    {
      prev_aff_labels_.markers[i].action = visualization_msgs::Marker::DELETE;
      prev_aff_labels_.markers[i].header.stamp = ros::Time::now ();
      labels.markers.push_back (prev_aff_labels_.markers[i]);
    }
  }

  prev_aff_labels_ = aff_labels_;

  if (pub_affordance_markers_.getNumSubscribers ())
    pub_affordance_markers_.publish (labels);
  return true;
}

bool
getSceneObjects (const al_msgs::Entities& entities, al_msgs::SceneObjects& scene_objects)
{
  uint n_objects = entities.entities.size ();
  for (uint i = 0; i < n_objects; i++)
  {
    if (!al::facilities::toInt (entities.entities[i].collision_object.id))
      continue;

    req_learned_shapes.feature_vector = entities.entities[i].feature_vector;

    if (srv_cl_categories_.call (req_learned_shapes, res_learned_shapes))
    {
      al_msgs::Object object;
      object.object = entities.entities[i].collision_object;
      object.shapes = res_learned_shapes.shapes;
      scene_objects.objects.push_back (object);
    }
    else
      std::cout << "error while getting learned affordances of the object" << std::endl;
  }
  return true;
}

bool
getSceneAffordances (const al_msgs::Entities& entities, al_msgs::SceneAffordances& scene_affordances)
{
  uint n_objects = entities.entities.size ();
  for (uint i = 0; i < n_objects; i++)
  {
    if (!al::facilities::toInt (entities.entities[i].collision_object.id))
      continue;

    req_learned_effects.feature_vector = entities.entities[i].feature_vector;

    if (srv_cl_affordances_.call (req_learned_effects, res_learned_effects))
    {
      for (uint j = 0; j < res_learned_effects.effects.size (); j++)
      {
        std::cout << al::facilities::behaviorEnumToString (res_learned_effects.effects[j].arg) << "\t";
        std::cout << al::facilities::effectEnumToString (res_learned_effects.effects[j].effect) << "\t";
        std::cout << res_learned_effects.effects[j].prob << std::endl;
        map_effects[res_learned_effects.effects[j].prob] = res_learned_effects.effects[j];
      }

      //now pick top 5 different affordances
      //convert into an affordance message and publish
      it = map_effects.end ();
      al_msgs::Affordances affordances;
      while (it != map_effects.begin () && affordances.effects.size () < TOP_N_EFFECTS)
      {
        --it;
        bool same_effect_exists = false;
        for (uint j = 0; j < affordances.effects.size (); j++)
        {
          if (it->second.effect == affordances.effects[j].effect)
          {
            same_effect_exists = true;
            if (it->first > affordances.effects[j].prob)
            {
              affordances.effects[j].effect = it->second.effect;
              break;
            }
          }
        }
        if (!same_effect_exists)
        {
          affordances.effects.push_back (it->second);
        }
      }
      affordances.object = entities.entities[i].collision_object;
      std::cout << "n_affordances: " << (int)affordances.effects.size () << std::endl;
      scene_affordances.scene_affordances.push_back (affordances);
      map_effects.clear ();
    }
    else
    {
      std::cout << "error while getting learned affordances of the object" << std::endl;
    }
  }
  return true;
}

bool
getSceneFeatures (al_msgs::Entities& entities)
{
  srv_scene_.request.seconds_ago = 10;
  srv_scene_.request.header.stamp = ros::Time::now ();
  bool success = ros::service::call ("/get_scene", srv_scene_.request, srv_scene_.response);
  if (success)
  {
    entities = srv_scene_.response.scene.entities;
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
      }
    }
    else
      std::cout << "problem with feature extraction" << std::endl;
  }
  else
    std::cout << "problem with scene extraction" << std::endl;

  std::cout << "current_scene: " << std::endl;
  for (uint i = 0; i < entities.entities.size (); i++)
  {
    al_msgs::Entity entity = entities.entities[i];
    std::cout << "entity: " << entity.collision_object.id << " w/ feature_size: "
        << (int)entity.feature_vector.features.size () << std::endl;
  }

  return success;
}
