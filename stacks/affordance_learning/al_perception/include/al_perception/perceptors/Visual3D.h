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

#include "al_perception/perceptors/Perceptor.h"
#include "al_perception/feature_extractors/SurfaceFExtractor.h"
#include "al_perception/feature_extractors/SpatialFExtractor.h"
#include "al_perception/feature_extractors/HumanFExtractor.h"
#include "al_perception/objects/Entity.h"

#include "al_msgs/Entities.h"
#include "al_msgs/CollisionObjects.h"
#include "al_msgs/CloudObjects.h"
#include "al_srvs/GetEntityVisualFeatures.h"
#include "al_srvs/GetEntitiesVisualFeatures.h"

//TODO: import this header properly

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

//#include "object_manipulation_msgs/FindClusterBoundingBox2.h"

//#include <pcl/features/integral_image_normal.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/ros/conversions.h>
#include "pcl_ros/filters/statistical_outlier_removal.h"
#include "pcl_ros/filters/radius_outlier_removal.h"
#include "pcl_ros/filters/passthrough.h"

#include "tabletop_object_detector/TabletopSegmentation.h"
#include "tabletop_collision_map_processing/collision_map_interface.h"

#include "arm_navigation_msgs/CollisionObjectOperation.h"
#include "arm_navigation_msgs/Shape.h"

namespace al
{
  namespace perception
  {
    const std::string TABLETOP_SEGMENTATION_SRV_NAME = "/tabletop_segmentation";
    const std::string POINTCLOUD_FILTERED = "/pointcloud_filtered";
    const uint8_t N_PERSISTENT_SERVICE_CALL = 10;
    const uint8_t N_NORMAL_ESTIMATION_THREADS = 4;
    const uint MIN_HUGE_BOX_ID = 100;

    const double BOX_SIMILARITY_UPPER_VOL_THRESH = 1.25;
    const double BOX_SIMILARITY_LOWER_VOL_THRESH = 0.75;
    const double BOX_EXTEND_MAX_Z_THRESHOLD = 0.05;//boxes that are at most 5cm above the table will be extended to the table
    const double BOX_SIMILARITY_POSITION_THRESHOLD = 0.05;
    const double BOX_SIMILARITY_THRESHOLD = 0.02;
    const double BOX_SIMILARTY_X = 1;//depth of the box more prone to change (depending on the distance from the sensor)
    const double BOX_SIMILARTY_Y = 1;//width may change with the as depth if the object is rotated
    const double BOX_SIMILARTY_Z = 1;//height of the box most discriminative

    const uint16_t MIN_OBJECT_CLUSTER_SIZE = 40;

    class Visual3D : public Perceptor
    {
    public:
      Visual3D (ros::NodeHandle* nh);

      void
      init ();

      bool
      isReady ();

      //TODO: feature extractions according to the configurations
      int
      percept ();

      virtual
      ~Visual3D ();

    protected:
      ros::Subscriber sub_pointcloud_;
      ros::Publisher pub_pointcloud_filtered_;
      ros::Publisher pub_entities_;
      ros::Publisher pub_collision_objects_;
      ros::Publisher pub_collision_object_;
      //      ros::Publisher pub_cloud_objects_;
      ros::ServiceClient srv_cl_tabletop_detection_;
      ros::ServiceServer srv_get_entity_visual_features_;
      ros::ServiceServer srv_get_entities_visual_features_;

      sensor_msgs::PointCloud2 msg_pointcloud_filtered_;
      al_msgs::Entities entities_;
      al_msgs::CollisionObjects collision_objects_;
      al_msgs::CollisionObjects prev_collision_objects_;
      al_msgs::CloudObjects cloud_objects_;

      bool is_ready_;
      bool pointcloud_rcvd_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data_;

      tabletop_object_detector::TabletopSegmentation tabletop_segmentation_;
      tabletop_collision_map_processing::CollisionMapInterface collision_map_;

      std::vector<FeatureExtractor*> feature_extractors_;

      //holds filters for input pointcloud that are active
      std::vector<pcl::Filter<pcl::PointXYZ>*> filters_cloud_;
      pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter_cluster_clouds_;
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_data_;

      //stores the information about the bounding box dimensions of the entities during the whole interaction period
      //therefore it should be guaranteed to put distinct objects on the tabletop for a reasonable perception performance.
      std::map<int, geometry_msgs::Vector3> map_identified_box_dims_;

      void
      setClustersData (const std::vector<sensor_msgs::PointCloud>& clusters);

      void
      filterClusters (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters_data);

      al_msgs::Entity
      tableToEntityMsg (const tabletop_object_detector::Table& table);

      //uses prev_collision_objects to decide on the id/ids of the cluster
      bool
      cloudToEntitiesMsg (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data,
                          const tabletop_object_detector::Table& table, std::vector<al_msgs::Entity> &entities);

      //returns false if it cannot identify the entity from what is known. But assigns an id in anyway.
      bool
      identifyEntityFromKnownIds (al_msgs::Entity& entity);

      bool
      identifyEntityFromPrevIds (al_msgs::Entity& entity);

      void
      extractEntities (const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters_data,
                       const tabletop_object_detector::Table& table);

      bool
      srvGetEntityVisualFeatures (al_srvs::GetEntityVisualFeatures::Request &req,
                                  al_srvs::GetEntityVisualFeatures::Response &res);

      bool
      srvGetEntitiesVisualFeatures (al_srvs::GetEntitiesVisualFeatures::Request &req,
                                    al_srvs::GetEntitiesVisualFeatures::Response &res);

      void
      publishEntities ();

      void
      publishCollisionObjects ();

      void
      publishCloudObjects ();

      void
      pc2RcvdCallback (sensor_msgs::PointCloud2::ConstPtr pc2_msg);
    };
  }
}

#endif /* Visual3D_H_ */

