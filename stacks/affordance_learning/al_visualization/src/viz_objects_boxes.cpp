/*
 * viz_objects_boxes.cpp
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

#include "al_msgs/CollisionObjects.h"
#include "al_utils/al_utils.h"

#include "visualization_msgs/MarkerArray.h"

ros::NodeHandle* nh_;
ros::Publisher pub_bboxes_;
ros::Publisher pub_bboxes_txts_;
al_msgs::CollisionObjects collision_objects_;
std::vector<int> prev_collision_objects_ids_;
visualization_msgs::MarkerArray bboxes_;
visualization_msgs::MarkerArray prev_bboxes_;
visualization_msgs::MarkerArray bboxes_txts_;
visualization_msgs::MarkerArray prev_bboxes_txts_;
bool msg_collision_objects_rcvd_ = false;

void
colObjCallback (al_msgs::CollisionObjectsConstPtr collision_objects);

void
publishBoundingBoxes ();

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "viz_objects_boxes");
  nh_ = new ros::NodeHandle ("~");
  std::string input_objects_;
  if (!nh_->getParam ("input_objects", input_objects_))
  {
    ROS_WARN("there is no such parameter as <input_objects>");
    input_objects_ = "/perception/collision_objects";
  }
  ros::Subscriber sub_col_obj_;
  sub_col_obj_ = nh_->subscribe<al_msgs::CollisionObjects> (input_objects_.c_str (), 1, &colObjCallback);

  pub_bboxes_ = nh_->advertise<visualization_msgs::MarkerArray> ("bboxes", 5);
  pub_bboxes_txts_ = nh_->advertise<visualization_msgs::MarkerArray> ("bboxes_txts", 5);

  ros::Rate r (30);//objects can be refreshed at most @30hz
  while (nh_->ok ())
  {
    if (msg_collision_objects_rcvd_)
    {
      msg_collision_objects_rcvd_ = false;
      publishBoundingBoxes ();
    }
    ros::spinOnce ();
    r.sleep ();
  }
  return 0;
}

void
colObjCallback (al_msgs::CollisionObjectsConstPtr collision_objects)
{
  msg_collision_objects_rcvd_ = true;
  collision_objects_ = *collision_objects;
  std::cout << "collision_objects_.size: " << (int)collision_objects_.collision_objects.size () << std::endl;
  for (uint i = 0; i < collision_objects_.collision_objects.size (); i++)
  {
    std::cout <<"id: "<< collision_objects_.collision_objects[i].id << std::endl;
  }
  //  std::cout<<collision_objects_.collision_objects.size()<<std::endl;
}

void
publishBoundingBoxes ()
{
  //no one interested in bbox and bbox_txt visualization, just quit
  if (pub_bboxes_.getNumSubscribers () == 0 && pub_bboxes_txts_.getNumSubscribers () == 0)
    return;

  bboxes_.markers.resize (collision_objects_.collision_objects.size ());
  bboxes_txts_.markers.resize (collision_objects_.collision_objects.size ());

  std::vector<bool> used_prev_collision_objects (prev_collision_objects_ids_.size (), false);
  for (uint8_t i = 0; i < collision_objects_.collision_objects.size (); i++)
  {
    for (uint8_t j = 0; j < prev_collision_objects_ids_.size (); j++)
    {
      if (collision_objects_.collision_objects[i].id == al::facilities::toString (prev_collision_objects_ids_[j]))
      {
        used_prev_collision_objects[j] = true;
        break;
      }
    }

    visualization_msgs::Marker marker;
    marker.id = al::facilities::toInt (collision_objects_.collision_objects[i].id);
    marker.header.stamp = ros::Time::now ();
    marker.header.frame_id = collision_objects_.collision_objects[i].header.frame_id;
    marker.ns = nh_->getNamespace () + "/bboxes";
    marker.action = visualization_msgs::Marker::ADD;
    //    marker.header = collision_objects_.collision_objects[i].header;
    //    marker.type = collision_objects_.collision_objects[i].shapes[0].type;
    marker.type = arm_navigation_msgs::Shape::BOX;
    marker.lifetime = ros::Duration (1.0);
    marker.color = al::viz::colorize (marker.id, 0.5);
    try
    {
      marker.pose = collision_objects_.collision_objects[i].poses[0];
    }
    catch (std::exception& e)
    {
      //tried to access a collision_object pose that doesn't exist as a vector element!
      ROS_ERROR("%s", e.what());
      continue;
    }
    try
    {
      marker.scale.x = collision_objects_.collision_objects[i].shapes[0].dimensions[0];
      marker.scale.y = collision_objects_.collision_objects[i].shapes[0].dimensions[1];
      marker.scale.z = collision_objects_.collision_objects[i].shapes[0].dimensions[2];
    }
    catch (std::exception& e)
    {
      //tried to access a collision_object dimension that doesn't exist as a vector element!
      ROS_ERROR("%s", e.what());
      continue;
    }
    bboxes_.markers[i] = marker;

    marker.ns = nh_->getNamespace () + "/bboxes_txts";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    if (marker.id)
      marker.text = collision_objects_.collision_objects[i].id;
    marker.pose.position.z += (marker.scale.z / 2.0 + 0.05);
    marker.scale.x = 0.5;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;
    bboxes_txts_.markers[i] = marker;
  }

  for (uint i = 0; i < used_prev_collision_objects.size (); i++)
  {
    if (!used_prev_collision_objects[i])
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now ();
      marker.header.frame_id = "base_footprint";
      marker.ns = nh_->getNamespace () + "/bboxes";
      marker.action = visualization_msgs::Marker::DELETE;
      marker.id = prev_collision_objects_ids_[i];
      bboxes_.markers.push_back (marker);

      marker.ns = nh_->getNamespace () + "/bboxes_txts";
      bboxes_txts_.markers.push_back (marker);
    }
  }

  //  for (uint i = 0; i < bboxes_.markers.size (); i++)
  //    std::cout << bboxes_.markers[i] << std::endl;

  prev_collision_objects_ids_.resize (collision_objects_.collision_objects.size ());
  for (uint i = 0; i < prev_collision_objects_ids_.size (); i++)
    prev_collision_objects_ids_[i] = al::facilities::toInt (collision_objects_.collision_objects[i].id);

  if (pub_bboxes_.getNumSubscribers ())
    pub_bboxes_.publish (bboxes_);

  if (pub_bboxes_txts_.getNumSubscribers ())
    pub_bboxes_txts_.publish (bboxes_txts_);
}
