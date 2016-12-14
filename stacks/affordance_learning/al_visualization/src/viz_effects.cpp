/*
 * viz_effects.cpp
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

void
affordancesCompCallback (al_msgs::AffordancesComp::ConstPtr affordances_comp);

void
affordancesCallback (al_msgs::Affordances::ConstPtr affordances);

void
updateAffordanceMarkers ();

ros::NodeHandle* nh_;
ros::Subscriber sub_affordances_comp_;
ros::Subscriber sub_affordances_;
ros::Publisher pub_affordance_markers_;

visualization_msgs::MarkerArray aff_labels_;
visualization_msgs::MarkerArray prev_aff_labels_;
al_msgs::Affordances affordances_;
bool start_ = false;

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_viz_affordances");
  nh_ = new ros::NodeHandle ();

  sub_affordances_ = nh_->subscribe ("/affordances", 1, &affordancesCallback);
  sub_affordances_comp_ = nh_->subscribe ("affordances_comp", 1, &affordancesCompCallback);
  pub_affordance_markers_ = nh_->advertise<visualization_msgs::MarkerArray> ("affordance_labels", 1);

  ros::Rate r (50);
  while (nh_->ok ())
  {
    ros::spinOnce ();
    r.sleep ();
  }
}

void
affordancesCompCallback (al_msgs::AffordancesComp::ConstPtr affordances_comp)
{
  affordances_.object.id = al::facilities::toString (affordances_comp->entity_id);
  affordances_.behavior_id = affordances_comp->behavior_id;
  affordances_.effects = affordances_comp->effects;
  affordances_.object.poses.resize (1);
  affordances_.object.poses[0].position.x = affordances_comp->object_pos[0];
  affordances_.object.poses[0].position.y = affordances_comp->object_pos[1];
  affordances_.object.poses[0].position.z = affordances_comp->object_pos[2];

  arm_navigation_msgs::Shape shape;
  shape.type = arm_navigation_msgs::Shape::BOX;
  shape.dimensions.resize (3);
  shape.dimensions[0] = affordances_comp->object_dims[0];
  shape.dimensions[1] = affordances_comp->object_dims[1];
  shape.dimensions[2] = affordances_comp->object_dims[2];
  affordances_.object.shapes.resize (1);
  affordances_.object.shapes[0] = shape;

  updateAffordanceMarkers ();
}

void
affordancesCallback (al_msgs::Affordances::ConstPtr affordances)
{
  affordances_ = *affordances;

  updateAffordanceMarkers ();
}

void
updateAffordanceMarkers ()
{
  std::cout << "updating" << std::endl;

  if (affordances_.effects.size ())
    aff_labels_.markers.resize (affordances_.effects.size () + 1);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_footprint";
  marker.header.stamp = ros::Time::now ();
  marker.ns = "aff_label_texts";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration (5);

  geometry_msgs::Point start;
  start.x = affordances_.object.poses[0].position.x;
  start.y = affordances_.object.poses[0].position.y;
  start.z = affordances_.object.poses[0].position.z;

  geometry_msgs::Point end;
  end.x = -0.60;
  if (affordances_.object.poses[0].position.y > 0)
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

  aff_labels_.markers[0] = marker;

  for (uint8_t i = 0; i < affordances_.effects.size (); i++)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "aff_label_texts";
    marker.id = i + 1;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = aff_labels_.markers[0].points[1].x;
    marker.pose.position.y = aff_labels_.markers[0].points[1].y;
    marker.pose.position.z = aff_labels_.markers[0].points[1].z + 0.09 * (i + 1);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    //    float prob = al::math::fPrecision (affordances_.effects[i].prob, 1, 2);
    //    std::stringstream s;
    //    s << prob;

    std::stringstream s;
    s << (int)(affordances_.effects[i].prob * 100);

    std::string text = al::facilities::effectEnumToString (affordances_.effects[i].effect) + "\t" + "( " + s.str ()
        + "% )";

    marker.text = text;
    marker.scale.x = 0.15;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    aff_labels_.markers[i + 1] = marker;
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
}
