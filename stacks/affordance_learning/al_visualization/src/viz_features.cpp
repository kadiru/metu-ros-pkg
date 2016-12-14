/*
 * viz_features.cpp
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
#include "visualization_msgs/MarkerArray.h"

#include <pcl/ros/conversions.h>
#include "pcl/point_types.h"

#include "al_msgs/Features.h"
#include "al_utils/al_utils.h"

ros::NodeHandle* nh_;
ros::Subscriber sub_features_;

ros::Publisher pub_viz_normals_;
ros::Publisher pub_viz_min_curvs_;
ros::Publisher pub_viz_max_curvs_;
ros::Publisher pub_viz_gauss_curvs_;
ros::Publisher pub_viz_shape_indices_;
ros::Publisher pub_viz_labels_;

al_msgs::Features features_msg_;

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data_;

bool msg_entities_rcvd_ = false;

void
featuresCallback (al_msgs::FeaturesConstPtr features);

const double C_ZERO = 0.03;
const double H_ZERO = C_ZERO;
const double K_ZERO = H_ZERO * H_ZERO;

//const double MARKER_SCALE = 0.0005;//for dense bunny
//const double MARKER_SCALE = 0.003;//small bunny
const double MARKER_SCALE = 0.001;//banana
std_msgs::ColorRGBA
labelCurvature (const double h, const double k, const double s, const double c);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_viz_features");
  nh_ = new ros::NodeHandle ("~");

  sub_features_ = nh_->subscribe ("/features", 1, &featuresCallback);
  pub_viz_labels_ = nh_->advertise<visualization_msgs::Marker> ("viz_labels", 10);
  pub_viz_shape_indices_ = nh_->advertise<visualization_msgs::Marker> ("viz_shape_indices", 10);
  pub_viz_gauss_curvs_ = nh_->advertise<visualization_msgs::Marker> ("viz_gaussian_curvs", 10);

  pointcloud_data_.reset (new pcl::PointCloud<pcl::PointXYZ>);

  ros::Rate r (30);//objects can be refreshed at most @30hz
  while (nh_->ok ())
  {
    if (msg_entities_rcvd_)
    {
      msg_entities_rcvd_ = false;

      //now do the visualization stuff
      pcl::fromROSMsg (features_msg_.cloud, *pointcloud_data_);

      visualization_msgs::Marker m_gaussian_curvatures;
      visualization_msgs::Marker m_mean_curvatures;
      visualization_msgs::Marker m_shape_indices;
      visualization_msgs::Marker m_curvedness;
      visualization_msgs::Marker m_labels;

      m_gaussian_curvatures.pose.orientation.w = 1.0;
      m_gaussian_curvatures.action = visualization_msgs::Marker::ADD;
      m_gaussian_curvatures.type = visualization_msgs::Marker::POINTS;
      m_gaussian_curvatures.points.resize (pointcloud_data_->points.size ());
      m_gaussian_curvatures.colors.resize (pointcloud_data_->points.size ());
      m_gaussian_curvatures.header.frame_id = "base_footprint";
      m_gaussian_curvatures.header.stamp = ros::Time::now ();
      m_gaussian_curvatures.lifetime = ros::Duration ();
      m_gaussian_curvatures.id = 1; //this doesn't do any identification since there is only one point in each ns
      m_gaussian_curvatures.ns = "gaussian_curvaures";
      m_gaussian_curvatures.color.a = 1.0;
      m_gaussian_curvatures.scale.x = MARKER_SCALE;
      m_gaussian_curvatures.scale.y = MARKER_SCALE;
      m_gaussian_curvatures.scale.z = 1.0;

      m_shape_indices.pose.orientation.w = 1.0;
      m_shape_indices.action = visualization_msgs::Marker::ADD;
      m_shape_indices.type = visualization_msgs::Marker::POINTS;
      m_shape_indices.points.resize (pointcloud_data_->points.size ());
      m_shape_indices.colors.resize (pointcloud_data_->points.size ());
      m_shape_indices.header.frame_id = "base_footprint";
      m_shape_indices.header.stamp = ros::Time::now ();
      m_shape_indices.lifetime = ros::Duration ();
      m_shape_indices.id = 1; //this doesn't do any identification since there is only one point in each ns
      m_shape_indices.ns = "shape_indices";
      m_shape_indices.color.a = 1.0;
      m_shape_indices.scale.x = MARKER_SCALE;
      m_shape_indices.scale.y = MARKER_SCALE;
      m_shape_indices.scale.z = 1.0;

      m_mean_curvatures.pose.orientation.w = 1.0;
      m_mean_curvatures.action = visualization_msgs::Marker::ADD;
      m_mean_curvatures.type = visualization_msgs::Marker::POINTS;
      m_mean_curvatures.points.resize (pointcloud_data_->points.size ());
      m_mean_curvatures.colors.resize (pointcloud_data_->points.size ());
      m_mean_curvatures.header.frame_id = "base_footprint";
      m_mean_curvatures.header.stamp = ros::Time::now ();
      m_mean_curvatures.lifetime = ros::Duration ();
      m_mean_curvatures.id = 1; //this doesn't do any identification since there is only one point in each ns
      m_mean_curvatures.ns = "mean_curvatures";
      m_mean_curvatures.color.a = 1.0;
      m_mean_curvatures.scale.x = MARKER_SCALE;
      m_mean_curvatures.scale.y = MARKER_SCALE;
      m_mean_curvatures.scale.z = 1.0;

      m_curvedness.pose.orientation.w = 1.0;
      m_curvedness.action = visualization_msgs::Marker::ADD;
      m_curvedness.type = visualization_msgs::Marker::POINTS;
      m_curvedness.points.resize (pointcloud_data_->points.size ());
      m_curvedness.colors.resize (pointcloud_data_->points.size ());
      m_curvedness.header.frame_id = "base_footprint";
      m_curvedness.header.stamp = ros::Time::now ();
      m_curvedness.lifetime = ros::Duration ();
      m_curvedness.id = 1; //this doesn't do any identification since there is only one point in each ns
      m_curvedness.ns = "mean_curvatures";
      m_curvedness.color.a = 1.0;
      m_curvedness.scale.x = MARKER_SCALE;
      m_curvedness.scale.y = MARKER_SCALE;
      m_curvedness.scale.z = 1.0;

      m_labels.pose.orientation.w = 1.0;
      m_labels.action = visualization_msgs::Marker::ADD;
      m_labels.type = visualization_msgs::Marker::POINTS;
      m_labels.points.resize (pointcloud_data_->points.size ());
      m_labels.colors.resize (pointcloud_data_->points.size ());
      m_labels.header.frame_id = "base_footprint";
      m_labels.header.stamp = ros::Time::now ();
      m_labels.lifetime = ros::Duration ();
      m_labels.id = 1; //this doesn't do any identification since there is only one point in each ns
      m_labels.ns = "labels";
      m_labels.color.a = 1.0;
      m_labels.scale.x = MARKER_SCALE;
      m_labels.scale.y = MARKER_SCALE;
      m_labels.scale.z = 1.0;

      std::vector<float> h = features_msg_.mean_curvatures;
      std::vector<float> k = features_msg_.gaussian_curvatures;
      std::vector<float> s = features_msg_.shape_indices;
      std::vector<float> c = features_msg_.curvedness;

      for (uint j = 0; j < pointcloud_data_->points.size (); j++)
      {
        geometry_msgs::Point pos;
        pos.x = pointcloud_data_->points[j].x;
        pos.y = pointcloud_data_->points[j].y;
        pos.z = pointcloud_data_->points[j].z;

        m_gaussian_curvatures.points[j] = pos;
        m_mean_curvatures.points[j] = pos;
        m_shape_indices.points[j] = pos;
        m_curvedness.points[j] = pos;
        m_labels.points[j] = pos;

        m_labels.colors[j] = labelCurvature (h[j], k[j], s[j], c[j]);
        m_shape_indices.colors[j] = al::viz::cvtHSVToRGB (240 * (1 - ((s[j] + 1) / 2)), 1.0, 1.0);

        if (k[j] > 2)
        {
          std::cout << k[j] << " ";
          k[j] = 2;
        }
        else if (k[j] < -2)
        {
          std::cout << k[j] << " ";
          k[j] = -2;
        }

        m_gaussian_curvatures.colors[j] = al::viz::cvtHSVToRGB (240 * (1 - ((k[j] + 2) / 4)), 1.0, 1.0);
      }
      if (pub_viz_labels_.getNumSubscribers ())
      {
        std::cout << "published labeled points" << std::endl;
        pub_viz_labels_.publish (m_labels);
      }
      if (pub_viz_shape_indices_.getNumSubscribers ())
      {
        std::cout << "published shape indices" << std::endl;
        pub_viz_shape_indices_.publish (m_shape_indices);
      }
      if (pub_viz_gauss_curvs_.getNumSubscribers ())
      {
        std::cout << "published shape indices" << std::endl;
        pub_viz_gauss_curvs_.publish (m_gaussian_curvatures);
      }
    }
    ros::spinOnce ();
    r.sleep ();
  }

  return 0;
}

std_msgs::ColorRGBA
labelCurvature (const double h, const double k, const double s, const double c)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;

  if (c > C_ZERO)
  {
    if (h < -H_ZERO)
    {
      if (k > K_ZERO && s >= 5.0 / 8 && s <= 1) //BLUE
      {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
      }
      else if (k < K_ZERO && s >= 3.0 / 8 && s <= 5.0 / 8) //MAGENTA
      {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
      }
      else if (k < -K_ZERO && s >= 3.0 / 16 && s <= 3.0 / 8) //RED
      {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
      }
    }
    else if (h < H_ZERO)
    {
      if (k < -K_ZERO && s >= -3.0 / 16 && s <= 3.0 / 16) //ORANGE
      {
        color.r = 1.0;
        color.g = 0.5;
        color.b = 0.0;
      }
    }
    else if (h > H_ZERO)
    {
      if (k > K_ZERO && s >= -1 && s <= -5.0 / 8) //CYAN
      {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
      }
      else if (k < K_ZERO && s >= -5.0 / 8 && s <= -3.0 / 8) //GREEN
      {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
      }
      else if (k < -K_ZERO && s >= -3.0 / 16 && s <= 3.0 / 16) //YELLOW
      {
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
      }
    }
  }
  else//PLANAR
  {
    if (h < H_ZERO && k < K_ZERO) //GREY
    {
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.5;
    }
  }
  return color;
}

void
featuresCallback (al_msgs::FeaturesConstPtr features)
{
  std::cout << "features rcvd" << std::endl;
  msg_entities_rcvd_ = true;
  features_msg_ = *features;
}
