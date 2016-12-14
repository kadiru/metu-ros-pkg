/*
 * viz_objects_surfaces.cpp
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

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "visualization_msgs/MarkerArray.h"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "al_msgs/Entities.h"
#include "al_perception/objects/Entity.h"

#include "ros/ros.h"
#include "vector"

ros::NodeHandle* nh_;
ros::Subscriber sub_entities_;
ros::Publisher pub_viz_normals_;
ros::Publisher pub_viz_min_curvs_;
ros::Publisher pub_viz_max_curvs_;
ros::Publisher pub_viz_gauss_curvs_;
ros::Publisher pub_viz_shape_indices_;
al_msgs::Entities entities_;
std::vector<int> prev_ids_;
std::vector<int> prev_cloud_sizes_;
std::vector<bool> prev_cloud_used_;
pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_;
pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce_;
pcl::PointCloud<pcl::PrincipalCurvatures> princip_curves_;

visualization_msgs::MarkerArray markers_normals_;
visualization_msgs::MarkerArray markers_min_curvs_;
visualization_msgs::MarkerArray markers_max_curvs_;
visualization_msgs::MarkerArray markers_gauss_curvs_;
visualization_msgs::MarkerArray markers_shape_indices_;

bool msg_entities_rcvd_ = false;

const int MIN_OBJECT_N_POINTS = 50;

void
entitiesCallback (al_msgs::EntitiesConstPtr entities);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_viz_objects_surfaces");
  nh_ = new ros::NodeHandle ("~");

  sub_entities_ = nh_->subscribe ("/perception/entities", 1, &entitiesCallback);
  pub_viz_normals_ = nh_->advertise < visualization_msgs::MarkerArray > ("viz_objects_normals", 1);
  pub_viz_min_curvs_ = nh_->advertise < visualization_msgs::MarkerArray > ("viz_objects_min_curvs", 1);
  pub_viz_max_curvs_ = nh_->advertise < visualization_msgs::MarkerArray > ("viz_objects_max_curvs", 1);
  //  pub_viz_gauss_curvs_ = nh_->advertise<visualization_msgs::MarkerArray> ("viz_objects_gauss_curvs", 1);
  pub_viz_shape_indices_ = nh_->advertise < visualization_msgs::MarkerArray > ("viz_objects_shape_indices", 1);

  pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  ne_.setSearchMethod (tree);
  ne_.setViewPoint (0, 0, 1.2);
  //  ne_.setKSearch (10);
  ne_.setRadiusSearch (0.02);
  pce_.setSearchMethod (tree);
  pce_.setRadiusSearch (0.02);
  //  pce_.setKSearch()
  //  pcl::PointCloud<pcl::PointXYZ>::ConstPtr positions = pce_.getInputCloud ();
  //  pcl::PointCloud<pcl::Normal>::ConstPtr normals = pce_.getInputNormals ();
  //  pcl::PointCloud<pcl::PrincipalCurvatures>::ConstPtr curves_ = princip_curves_.makeShared ();
  //  sensor_msgs::PointCloud2 pos, norm, curv;
  //  pcl::toROSMsg (*positions, pos);

  ros::Rate r (10); //objects can be refreshed at most @30hz
  while (nh_->ok ())
  {
    if (msg_entities_rcvd_)
    {
      msg_entities_rcvd_ = false;
      markers_normals_.markers.clear ();

      markers_min_curvs_.markers.clear (); // size: n_entities
      markers_max_curvs_.markers.clear (); // size: n_entities
      markers_shape_indices_.markers.clear (); // size: n_entities

      prev_cloud_used_.resize (prev_ids_.size ());
      for (uint k = 0; k < prev_cloud_used_.size (); k++)
        prev_cloud_used_[k] = false;

      uint marker_index_offset = 0;
      for (uint i = 0; i < entities_.entities.size (); i++)
      {
        al::perception::Entity entity = al::perception::fromEntityMsg (entities_.entities[i]);
        if (entity.processable ())
        {
          ne_.setInputCloud (entity.getCloudData ());
          ne_.compute (*pointcloud_normals);
          pce_.setInputCloud (entity.getCloudData ());
          pce_.setInputNormals (pointcloud_normals);
          pce_.compute (princip_curves_);

          std::vector<float> shape_indices (princip_curves_.points.size ());

          float min_shape_index = FLT_MAX;
          float max_shape_index = FLT_MIN;
          for (uint i = 0; i < princip_curves_.points.size (); i++)
          {
            shape_indices[i] = atan2 (princip_curves_.points[i].pc1 + princip_curves_.points[i].pc2,
                                      princip_curves_.points[i].pc1 - princip_curves_.points[i].pc2) / al::math::PI;

            if (shape_indices[i] < min_shape_index)
              min_shape_index = shape_indices[i];

            if (shape_indices[i] > max_shape_index)
              max_shape_index = shape_indices[i];
          }

          float min_min_curv = FLT_MAX;
          float max_min_curv = FLT_MIN;
          float min_max_curv = FLT_MAX;
          float max_max_curv = FLT_MIN;

          std::cout << "*************************\n";
          std::cout << "entity " << entity.getId () << std::endl;
          for (uint j = 0; j < princip_curves_.points.size (); j++)
          {
            std::cout << princip_curves_.points[j].pc1 << "\t" << princip_curves_.points[j].pc2 << std::endl;
            std::cout << princip_curves_.points[j].principal_curvature_x << "\t"
                << princip_curves_.points[j].principal_curvature_y << "\t"
                << princip_curves_.points[j].principal_curvature_z << std::endl;

            if (princip_curves_.points[j].pc1 > max_max_curv)
              max_max_curv = princip_curves_.points[j].pc1; //max curv

            if (princip_curves_.points[j].pc1 < min_max_curv)
              min_max_curv = princip_curves_.points[j].pc1; //max curv

            if (princip_curves_.points[j].pc2 > max_min_curv)
              max_min_curv = princip_curves_.points[j].pc2; //min curv

            if (princip_curves_.points[j].pc2 < min_min_curv)
              min_min_curv = princip_curves_.points[j].pc2; //min curv
          }

          std::cout << "min of min: " << min_min_curv << "\t" << "max of min: " << max_min_curv << std::endl;
          std::cout << "min of max: " << min_max_curv << "\t" << "max of max: " << max_max_curv << std::endl;

          //check if this object was there in the previous frame
          uint curr_marker_offset = 0;
          int id_index = -1;
          for (uint j = 0; j < prev_ids_.size (); j++)
          {
            if (prev_ids_[j] == entity.getId ())
            {
              prev_cloud_used_[j] = true;
              id_index = j;

              if ((uint)prev_cloud_sizes_[id_index] > pointcloud_normals->points.size ())
                curr_marker_offset = prev_cloud_sizes_[id_index];
              else
                curr_marker_offset = pointcloud_normals->points.size ();
              break;
            }
          }

          if (id_index == -1) //exists in the prev frame
            curr_marker_offset = pointcloud_normals->points.size ();

          marker_index_offset = markers_normals_.markers.size ();
          markers_normals_.markers.resize (marker_index_offset + curr_marker_offset);

          visualization_msgs::Marker marker_min_curvatures;
          visualization_msgs::Marker marker_max_curvatures;
          visualization_msgs::Marker marker_shape_indices;

          marker_min_curvatures.pose.orientation.w = 1.0;
          marker_min_curvatures.action = visualization_msgs::Marker::ADD;
          marker_min_curvatures.type = visualization_msgs::Marker::POINTS;
          marker_min_curvatures.points.resize (pointcloud_normals->points.size ());
          marker_min_curvatures.colors.resize (pointcloud_normals->points.size ());
          marker_min_curvatures.header.frame_id = "base_footprint";
          marker_min_curvatures.header.stamp = ros::Time::now ();
          marker_min_curvatures.lifetime = ros::Duration ();
          marker_min_curvatures.id = entity.getId (); //this doesn't do any identification since there is only one point in each ns
          marker_min_curvatures.ns = "viz_min_curvs/object_" + al::facilities::toString (marker_min_curvatures.id);
          marker_min_curvatures.color.a = 1.0;
          marker_min_curvatures.scale.x = 0.004;
          marker_min_curvatures.scale.y = 0.004;
          marker_min_curvatures.scale.z = 1.0;

          marker_shape_indices.pose.orientation.w = 1.0;
          marker_shape_indices.action = visualization_msgs::Marker::ADD;
          marker_shape_indices.type = visualization_msgs::Marker::POINTS;
          marker_shape_indices.points.resize (pointcloud_normals->points.size ());
          marker_shape_indices.colors.resize (pointcloud_normals->points.size ());
          marker_shape_indices.header.frame_id = "base_footprint";
          marker_shape_indices.header.stamp = ros::Time::now ();
          marker_shape_indices.lifetime = ros::Duration ();
          marker_shape_indices.id = entity.getId (); //this doesn't do any identification since there is only one point in each ns
          marker_shape_indices.ns = "viz_shape_indices/object_" + al::facilities::toString (marker_min_curvatures.id);
          marker_shape_indices.color.a = 1.0;
          marker_shape_indices.scale.x = 0.004;
          marker_shape_indices.scale.y = 0.004;
          marker_shape_indices.scale.z = 1.0;

          marker_max_curvatures.pose.orientation.w = 1.0;
          marker_max_curvatures.action = visualization_msgs::Marker::ADD;
          marker_max_curvatures.type = visualization_msgs::Marker::POINTS;
          marker_max_curvatures.points.resize (pointcloud_normals->points.size ());
          marker_max_curvatures.colors.resize (pointcloud_normals->points.size ());
          marker_max_curvatures.header.frame_id = "base_footprint";
          marker_max_curvatures.header.stamp = ros::Time::now ();
          marker_max_curvatures.lifetime = ros::Duration ();
          marker_max_curvatures.id = entity.getId (); //this doesn't do any identification since there is only one point in each ns
          marker_max_curvatures.ns = "viz_max_curvs/object_" + al::facilities::toString (marker_max_curvatures.id);
          marker_max_curvatures.color.a = 1.0;
          marker_max_curvatures.scale.x = 0.004;
          marker_max_curvatures.scale.y = 0.004;
          marker_max_curvatures.scale.z = 1.0;

          for (uint j = 0; j < pointcloud_normals->points.size (); j++)
          {
            geometry_msgs::Point pos;
            pos.x = entity.getCloudData ()->points[j].x;
            pos.y = entity.getCloudData ()->points[j].y;
            pos.z = entity.getCloudData ()->points[j].z;

            markers_normals_.markers[marker_index_offset + j].pose.position = pos;
            marker_min_curvatures.points[j] = pos;
            marker_max_curvatures.points[j] = pos;
            marker_shape_indices.points[j] = pos;

            //axis-angle rotation
            btVector3 marker_axis (1, 0, 0);
            btVector3 axis (pointcloud_normals->points[j].normal_x, pointcloud_normals->points[j].normal_y,
                            pointcloud_normals->points[j].normal_z);
            btQuaternion qt (marker_axis.cross (axis.normalize ()), marker_axis.angle (axis.normalize ()));
            geometry_msgs::Quaternion quat_msg;
            tf::quaternionTFToMsg (qt, quat_msg);
            markers_normals_.markers[marker_index_offset + j].pose.orientation = quat_msg;
            markers_normals_.markers[marker_index_offset + j].header.frame_id = "base_footprint";
            markers_normals_.markers[marker_index_offset + j].lifetime = ros::Duration (1);
            markers_normals_.markers[marker_index_offset + j].id = j;
            markers_normals_.markers[marker_index_offset + j].ns = "viz_normals/object_"
                + al::facilities::toString (entity.getId ());
            markers_normals_.markers[marker_index_offset + j].color = al::viz::colorize (entity.getId (), 1.0);

            marker_min_curvatures.colors[j] = al::viz::cvtHSVToRGB (
                240 * (1 - ((princip_curves_.points[j].pc2 - min_min_curv) / (max_min_curv - min_min_curv))), 1.0, 1.0);

            marker_max_curvatures.colors[j] = al::viz::cvtHSVToRGB (
                240 * (1 - ((princip_curves_.points[j].pc1 - min_max_curv) / (max_max_curv - min_max_curv))), 1.0, 1.0);

            marker_shape_indices.colors[j] = al::viz::cvtHSVToRGB (
                240 * (1 - ((shape_indices[j] - min_shape_index) / (max_shape_index - min_shape_index))), 1.0, 1.0);

            markers_normals_.markers[marker_index_offset + j].type = visualization_msgs::Marker::ARROW;
            markers_normals_.markers[marker_index_offset + j].scale.x = 0.02;
            markers_normals_.markers[marker_index_offset + j].scale.y = 0.02;
            markers_normals_.markers[marker_index_offset + j].scale.z = 0.02;
            markers_normals_.markers[marker_index_offset + j].action = visualization_msgs::Marker::ADD;
          }

          markers_min_curvs_.markers.push_back (marker_min_curvatures);
          markers_max_curvs_.markers.push_back (marker_max_curvatures);
          markers_shape_indices_.markers.push_back (marker_shape_indices);

          if (id_index != -1) //exists in the prev frame
          {
            //delete excessive previous arrow markers
            for (uint j = pointcloud_normals->points.size (); j < (uint)prev_cloud_sizes_[id_index]; j++)
            {
              markers_normals_.markers[marker_index_offset + j].header.frame_id = "base_footprint";
              markers_normals_.markers[marker_index_offset + j].id = j;
              markers_normals_.markers[j].action = visualization_msgs::Marker::DELETE;
            }
            prev_cloud_sizes_[id_index] = pointcloud_normals->points.size ();
          }
          else
          {
            prev_ids_.push_back (entity.getId ());
            prev_cloud_sizes_.push_back (entity.getCloudData ()->size ());
            prev_cloud_used_.push_back (true);
          }
        }
      }

      for (uint i = 0; i < prev_cloud_used_.size (); i++)
      {
        if (!prev_cloud_used_[i])
        {
          visualization_msgs::Marker marker_min_curvatures;
          visualization_msgs::Marker marker_max_curvatures;
          visualization_msgs::Marker marker_shape_indices;

          marker_min_curvatures.header.frame_id = "base_footprint";
          marker_min_curvatures.ns = "viz_min_curvs/object_" + al::facilities::toString (prev_ids_[i]);
          marker_min_curvatures.id = prev_ids_[i];
          marker_min_curvatures.action = visualization_msgs::Marker::DELETE;

          marker_max_curvatures.header.frame_id = "base_footprint";
          marker_max_curvatures.ns = "viz_max_curvs/object_" + al::facilities::toString (prev_ids_[i]);
          marker_max_curvatures.id = prev_ids_[i];
          marker_max_curvatures.action = visualization_msgs::Marker::DELETE;

          marker_shape_indices.header.frame_id = "base_footprint";
          marker_shape_indices.ns = "viz_shape_indices/object_" + al::facilities::toString (prev_ids_[i]);
          marker_shape_indices.id = prev_ids_[i];
          marker_shape_indices.action = visualization_msgs::Marker::DELETE;

          markers_min_curvs_.markers.push_back (marker_min_curvatures);
          markers_max_curvs_.markers.push_back (marker_max_curvatures);
          markers_shape_indices_.markers.push_back (marker_shape_indices);

          marker_index_offset = markers_normals_.markers.size ();
          markers_normals_.markers.resize (marker_index_offset + prev_cloud_sizes_[i]);
          for (uint j = marker_index_offset; j < markers_normals_.markers.size (); j++)
          {
            markers_normals_.markers[j].header.frame_id = "base_footprint";
            markers_normals_.markers[j].ns = "viz_normals/object_" + al::facilities::toString (prev_ids_[i]);
            markers_normals_.markers[j].id = j - marker_index_offset;
            markers_normals_.markers[j].action = visualization_msgs::Marker::DELETE;
          }
        }
      }

      for (uint i = 0; i < prev_cloud_used_.size (); i++)
      {
        if (!prev_cloud_used_[i])
        {
          prev_cloud_used_.erase (prev_cloud_used_.begin () + i);
          prev_cloud_sizes_.erase (prev_cloud_sizes_.begin () + i);
          prev_ids_.erase (prev_ids_.begin () + i);
          --i;
        }
      }

      if (pub_viz_normals_.getNumSubscribers ())
        pub_viz_normals_.publish (markers_normals_);
      if (pub_viz_min_curvs_.getNumSubscribers ())
        pub_viz_min_curvs_.publish (markers_min_curvs_);
      if (pub_viz_max_curvs_.getNumSubscribers ())
        pub_viz_max_curvs_.publish (markers_max_curvs_);
      if (pub_viz_shape_indices_.getNumSubscribers ())
        pub_viz_shape_indices_.publish (markers_shape_indices_);
    }
    ros::spinOnce ();
    r.sleep ();
  }
  return 0;
}

void
entitiesCallback (al_msgs::EntitiesConstPtr entities)
{
  msg_entities_rcvd_ = true;
  entities_ = *entities;
}

