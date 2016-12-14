/*
 * Visual3D.h
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

#ifndef Visual3D_H_
#define Visual3D_H_

#include "al_perception2/perceptors/Perceptor.h"
//#include "al_perception2/feature_extractors/SurfaceFExtractor.h"
#include "al_perception2/feature_extractors/SpatialFExtractor.h"
#include "al_perception2/feature_extractors/HumanFExtractor.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include "al_msgs/Table.h"

//#include "object_manipulation_msgs/FindClusterBoundingBox2.h"

//#include <pcl/features/integral_image_normal.h>
#include <pcl-1.6/pcl/features/fpfh_omp.h>
#include <pcl-1.6/pcl/ros/conversions.h>
#include "pcl-1.6/pcl/filters/statistical_outlier_removal.h"
#include "pcl-1.6/pcl/filters/radius_outlier_removal.h"
#include "pcl-1.6/pcl/filters/passthrough.h"
#include "pcl-1.6/pcl/search/kdtree.h"
#include "pcl-1.6/pcl/segmentation/sac_segmentation.h"
#include "pcl-1.6/pcl/filters/project_inliers.h"
#include "pcl-1.6/pcl/surface/convex_hull.h"
#include <pcl-1.6/pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl-1.6/pcl/segmentation/extract_clusters.h>
#include "pcl-1.6/pcl/common/transforms.h"

#include "pcl_ros/transforms.h"

#include "arm_navigation_msgs/Shape.h"

namespace al
{
  namespace perception
  {
    const std::string POINTCLOUD_FILTERED = "/pointcloud_filtered";

    enum DetectionResult
    {
      NO_CLOUD_RECEIVED = 1, NO_TABLE = 2, OTHER_ERROR = 3, SUCCESS = 4
    };

    typedef pcl::PointXYZRGB Point;
    typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;

    class Visual3D : public Perceptor
    {
    public:
      Visual3D (ros::NodeHandle* nh);

      void
      init ();

      bool
      isReady ();

      //TODO: feature extractions according to the configurations
      //TODO: convert everything to XYZRGB including all features extractors etc.
      //TODO:
      int
      percept ();

      virtual
      ~Visual3D ();

    protected:
      ros::Subscriber sub_pointcloud_;
      ros::Publisher pub_pointcloud_filtered_;

      sensor_msgs::PointCloud2 msg_pointcloud_filtered_;

      bool is_ready_;
      bool pointcloud_rcvd_;
      pcl::PointCloud<Point>::Ptr pointcloud_data_;
      std::vector<FeatureExtractor*> feature_extractors_;

      //holds filters for input pointcloud that are active
      std::vector<pcl::Filter<Point>*> filters_cloud_;
      pcl::RadiusOutlierRemoval<Point>* filter_cluster_clouds_;
      std::vector<pcl::PointCloud<Point>::Ptr> clusters_data_;

      //stores the information about the bounding box dimensions of the entities during the whole interaction period
      //therefore it should be guaranteed to put distinct objects on the tabletop for a reasonable perception performance.
      std::map<int, geometry_msgs::Vector3> map_identified_box_dims_;

      double plane_detection_voxel_x_;
      double plane_detection_voxel_y_;
      double plane_detection_voxel_z_;

      double clustering_voxel_x_;
      double clustering_voxel_y_;
      double clustering_voxel_z_;

      double min_z_limit;
      double max_z_limit;

      double cluster_distance_;
      double min_cluster_size_;

      double inlier_threshold_;
      bool flatten_table_;

      tf::TransformListener listener_;

      void
      pc2RcvdCallback (sensor_msgs::PointCloud2::ConstPtr pc2_msg);

      int
      extractTableAndObjects (al_msgs::Table & table, std::vector<pcl::PointCloud<Point>::Ptr>& objects);

      bool
      tableMsgToPointCloud (al_msgs::Table &table, std::string frame_id, pcl::PointCloud<Point>::Ptr &table_hull);

    };
  }
}

#endif /* Visual3D_H_ */

