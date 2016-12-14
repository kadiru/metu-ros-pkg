/*
 * SurfaceFExtractor.cpp
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
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
 *     * Neither the name of Kovan Lab nor the names of its
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

#include "al_perception/feature_extractors/SurfaceFExtractor.h"

namespace al
{
  namespace perception
  {
    SurfaceFExtractor::SurfaceFExtractor (ros::NodeHandle* nh) :
        FeatureExtractor (nh)
    {
      enable_normals_ = false;
      enable_curvatures_ = false;

      pce_.setKSearch (15);
    }

    SurfaceFExtractor::~SurfaceFExtractor ()
    {
      // TODO Auto-generated destructor stub
    }

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>*
    SurfaceFExtractor::getNormalEstimator ()
    {
      return &ne_;
    }

    void
    SurfaceFExtractor::extract (al::perception::Entity &entity)
    {
      //do nothing, probably wrong configuration
      if (!enable_curvatures_ && !enable_normals_)
        return;

      //in any case surface normals should be calculated
      pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals (new pcl::PointCloud<pcl::Normal>);
//      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne_.setSearchMethod (tree);
      ne_.setViewPoint (0, 0, 1.2); //this could be done more informed by getting this information from a topic

      if (entity.processable ())
      {
        ne_.setInputCloud (entity.getCloudData ());
        ne_.compute (*pointcloud_normals);

        if (enable_normals_)
        {
          cv::Mat normals_zen = cv::Mat (1, pointcloud_normals->points.size (), CV_32F);
          cv::Mat normals_azi = cv::Mat (1, pointcloud_normals->points.size (), CV_32F);

          for (uint16_t j = 0; j < pointcloud_normals->points.size (); j++)
          {
            double n_x = pointcloud_normals->points[j].normal_x;
            double n_y = pointcloud_normals->points[j].normal_y;
            double n_z = pointcloud_normals->points[j].normal_z;

            //always returns [0, PI]
            normals_zen.at<float> (0, j) = (atan2 (sqrt (n_x * n_x + n_y * n_y), n_z)) * 180 / al::math::PI;

            //may return [-PI, PI]
            normals_azi.at<float> (0, j) = (atan2 (n_y, n_x)) * 180 / al::math::PI;
            if (normals_azi.at<float> (0, j) < 0)
              normals_azi.at<float> (0, j) += 360;
          }

          al_msgs::Feature feature_zen;
          feature_zen.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
          feature_zen.range_min = 0;
          feature_zen.range_max = 360;
          feature_zen.n_hist_bins = N_NORMAL_HISTOGRAM_BINS;
          feature_zen.type = al_msgs::Feature::NORMAL_ZEN;

          calcFeature (feature_zen, normals_zen);

          al_msgs::Feature feature_azi;
          feature_azi.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
          feature_azi.range_min = 0;
          feature_azi.range_max = 360;
          feature_azi.n_hist_bins = N_NORMAL_HISTOGRAM_BINS;
          feature_azi.type = al_msgs::Feature::NORMAL_AZI;

          calcFeature (feature_azi, normals_azi);

          al_msgs::FeatureVector features_normal;
          features_normal.features.push_back (feature_azi);
          features_normal.features.push_back (feature_zen);

          entity.appendFeatures (features_normal);
        }
        if (enable_curvatures_)
        {
          //TODO: calculate surface curvatures here
          pce_.setSearchMethod (tree);
          pce_.setInputCloud (entity.getCloudData ());
          pce_.setInputNormals (pointcloud_normals);
          pce_.compute (princip_curves_);

          al_msgs::Feature feature_curvs_min;
          feature_curvs_min.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
          al_msgs::Feature feature_curvs_max;
          feature_curvs_max.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
          feature_curvs_min.range_min = 0;
          feature_curvs_min.range_max = 1;
          feature_curvs_min.n_hist_bins = N_CURVATURE_HISTOGRAM_BINS;
          feature_curvs_min.type = al_msgs::Feature::CURV_MIN;

          feature_curvs_max = feature_curvs_min;
          feature_curvs_max.type = al_msgs::Feature::CURV_MAX;

          cv::Mat curvs_min = cv::Mat (1, princip_curves_.points.size (), CV_32F);
          cv::Mat curvs_max = cv::Mat (1, princip_curves_.points.size (), CV_32F);

          for (uint16_t j = 0; j < princip_curves_.points.size (); j++)
          {

            curvs_min.at<float> (0, j) = princip_curves_.points[j].pc2;
            curvs_max.at<float> (0, j) = princip_curves_.points[j].pc1;
          }
          calcFeature (feature_curvs_min, curvs_min);
          calcFeature (feature_curvs_max, curvs_max);

          al_msgs::FeatureVector features_curvatures;
          features_curvatures.features.push_back (feature_curvs_min);
          features_curvatures.features.push_back (feature_curvs_max);
          entity.appendFeatures (features_curvatures);
        }

        std::vector<float> shape_indices (princip_curves_.points.size ());

        //extract shape index
        if (enable_curvatures_ && enable_normals_)
        {
          cv::Mat shape_indices = cv::Mat (1, princip_curves_.points.size (), CV_32F);
          for (uint i = 0; i < princip_curves_.points.size (); i++)
          {
            pcl::PrincipalCurvatures pcurves = princip_curves_.points[i];

            shape_indices.at<float> (0, i) = atan2 (pcurves.pc1 + pcurves.pc2, pcurves.pc1 - pcurves.pc2)
                / al::math::PI;
          }

          al_msgs::FeatureVector feature_shape_indices;
          al_msgs::Feature feature_sid; //shape index
          feature_sid.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
          feature_sid.range_min = -1.0;
          feature_sid.range_max = 1.0;
          feature_sid.type = al_msgs::Feature::SHAPE_INDEX;
          feature_sid.n_hist_bins = N_SHAPE_ID_HISTOGRAM_BINS;

          calcFeature (feature_sid, shape_indices);
          feature_shape_indices.features.push_back (feature_sid);
          entity.appendFeatures (feature_shape_indices);
        }
      }
    }

    void
    SurfaceFExtractor::enableNormals (bool enable)
    {
      enable_normals_ = enable;
    }

    void
    SurfaceFExtractor::enableCurvatures (bool enable)
    {
      enable_curvatures_ = enable;
    }
  }
}
