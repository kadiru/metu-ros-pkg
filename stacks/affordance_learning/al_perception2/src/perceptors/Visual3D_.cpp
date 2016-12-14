/*
 * Visual3D.cpp
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

#include "al_perception2/perceptors/Visual3D.h"

namespace al
{
  namespace perception
  {

    Visual3D::Visual3D (ros::NodeHandle* nh) :
      Perceptor (nh)
    {
      std::string cloud_in;
      if (!nh_->getParam ("/input_cloud", cloud_in))
        ROS_WARN("no such parameter as </input_cloud>");
      sub_pointcloud_ = nh_->subscribe (cloud_in.c_str (), 10, &Visual3D::pc2RcvdCallback, this);
      pub_pointcloud_filtered_ = nh_->advertise<sensor_msgs::PointCloud2> (POINTCLOUD_FILTERED, 0);

      pub_entities_ = nh_->advertise<al_msgs::Entities> ("entities", 10);
      pub_collision_objects_ = nh_->advertise<al_msgs::CollisionObjects> ("collision_objects", 10);
      pub_collision_object_ = nh_->advertise<arm_navigation_msgs::CollisionObject> ("/collision_object", 10);
      //      pub_cloud_objects_ = nh_->advertise<al_msgs::CloudObjects> ("cloud_objects", 10);

      srv_get_entity_visual_features_ = nh_->advertiseService ("/get_entity_visual_features",
                                                               &Visual3D::srvGetEntityVisualFeatures, this);
      srv_get_entities_visual_features_ = nh_->advertiseService ("/get_entities_visual_features",
                                                                 &Visual3D::srvGetEntitiesVisualFeatures, this);
    }

    void
    Visual3D::init ()
    {
      is_ready_ = false;
      pointcloud_rcvd_ = false;
      pointcloud_data_.reset (new pcl::PointCloud<pcl::PointXYZ>);
      filter_cluster_clouds_ = NULL;

      // if filter_outrem_cloud is active, configure it and add to the filters_cloud_ filter array
      bool filter_active = false;
      bool any_filter_active = false;
      if (nh_->getParam ("/Perceptors/Visual3D/filters/filter_outrem_cloud/is_active", filter_active))
      {
        if (filter_active)
        {
          pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter_outrem =
              new pcl::RadiusOutlierRemoval<pcl::PointXYZ> (false);
          double radius_search = 0.03;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_outrem_cloud/radius_search", radius_search))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_outrem_cloud/radius_search>");
          }
          int min_neighbors_in_radius = 45;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_outrem_cloud/min_neighbors_in_radius",
                              min_neighbors_in_radius))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_outrem_cloud/min_neighbors_in_radius>");
          }

          filter_outrem->setRadiusSearch (radius_search);
          filter_outrem->setMinNeighborsInRadius (min_neighbors_in_radius);

          filters_cloud_.push_back (filter_outrem);
        }
      }
      else
      {
        ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_outrem_cloud/is_active>");
      }
      any_filter_active = any_filter_active || filter_active;

      filter_active = false;
      if (nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_x_cloud/is_active", filter_active))
      {
        if (filter_active)
        {
          pcl::PassThrough<pcl::PointXYZ>* filter_pass_x = new pcl::PassThrough<pcl::PointXYZ> (false);
          double min_limit = 0.0;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_x_cloud/min_limit", min_limit))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_x_cloud/min_limit>");
          }
          double max_limit = 2.0;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_x_cloud/max_limit", max_limit))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_x_cloud/max_limit>");
          }

          filter_pass_x->setFilterLimits (min_limit, max_limit);

          filters_cloud_.push_back (filter_pass_x);
        }
      }
      else
      {
        ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_x_cloud/is_active>");
      }
      any_filter_active = any_filter_active || filter_active;

      filter_active = false;
      if (nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_y_cloud/is_active", filter_active))
      {
        if (filter_active)
        {
          pcl::PassThrough<pcl::PointXYZ>* filter_pass_y = new pcl::PassThrough<pcl::PointXYZ> (false);
          double min_limit = 0.0;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_y_cloud/min_limit", min_limit))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_y_cloud/min_limit>");
          }
          double max_limit = 2.0;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_y_cloud/max_limit", max_limit))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_y_cloud/max_limit>");
          }

          filter_pass_y->setFilterLimits (min_limit, max_limit);

          filters_cloud_.push_back (filter_pass_y);
        }
      }
      else
      {
        ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_y_cloud/is_active>");
      }
      any_filter_active = any_filter_active || filter_active;

      filter_active = false;
      if (nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_z_cloud/is_active", filter_active))
      {
        if (filter_active)
        {
          pcl::PassThrough<pcl::PointXYZ>* filter_pass_z = new pcl::PassThrough<pcl::PointXYZ> (false);
          double min_limit = 0.0;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_z_cloud/min_limit", min_limit))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_z_cloud/min_limit>");
          }
          double max_limit = 2.0;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_pass_z_cloud/max_limit", max_limit))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_z_cloud/max_limit>");
          }

          filter_pass_z->setFilterLimits (min_limit, max_limit);

          filters_cloud_.push_back (filter_pass_z);
        }
      }
      else
      {
        ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_pass_z_cloud/is_active>");
      }
      any_filter_active = any_filter_active || filter_active;

      filter_active = false;
      if (nh_->getParam ("/Perceptors/Visual3D/filters/filter_outrem_cluster_clouds/is_active", filter_active))
      {
        if (filter_active)
        {
          filter_cluster_clouds_ = new pcl::RadiusOutlierRemoval<pcl::PointXYZ> (false);
          double radius_search = 0.025;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_outrem_cluster_clouds/radius_search", radius_search))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_outrem_cluster_clouds/radius_search>");
          }
          int min_neighbors_in_radius = 45;
          if (!nh_->getParam ("/Perceptors/Visual3D/filters/filter_outrem_cluster_clouds/min_neighbors_in_radius",
                              min_neighbors_in_radius))
          {
            ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_outrem_cluster_clouds/min_neighbors_in_radius>");
          }

          filter_cluster_clouds_->setRadiusSearch (radius_search);
          filter_cluster_clouds_->setMinNeighborsInRadius (min_neighbors_in_radius);
        }
      }
      else
      {
        ROS_WARN("no such parameter as </Perceptors/Visual3D/filters/filter_outrem_cluster_clouds/is_active>");
      }

      bool f_normals_active_ = false;
      if (!nh_->getParam ("/Perceptors/Visual3D/features/normals/is_active", f_normals_active_))
        ROS_WARN("no such parameter as </Perceptors/Visual3D/features/normals/is_active>");

      //TODO: we probably don't need this to be a member, be sure about it
      bool f_curvatures_active_ = false;
      if (!nh_->getParam ("/Perceptors/Visual3D/features/curvatures/is_active", f_curvatures_active_))
        ROS_WARN("no such parameter as </Perceptors/Visual3D/features/curvatures/is_active>");

      bool f_spatial_active_ = false;
      if (!nh_->getParam ("/Perceptors/Visual3D/features/spatial/is_active", f_spatial_active_))
        ROS_WARN("no such parameter as </Perceptors/Visual3D/features/spatial/is_active>");

      bool f_human_active_ = false;
      if (!nh_->getParam ("/Perceptors/Visual3D/features/human/is_active", f_human_active_))
        ROS_WARN("no such parameter as </Perceptors/Visual3D/features/human/is_active>");
/*
      if (f_normals_active_ || f_curvatures_active_)
      {
        SurfaceFExtractor* surface_feature_extractor = new SurfaceFExtractor (nh_);
        surface_feature_extractor->enableNormals (f_normals_active_);
        surface_feature_extractor->enableCurvatures (f_curvatures_active_);

        if (f_normals_active_)
        {
          surface_feature_extractor->getNormalEstimator ()->setNumberOfThreads (N_NORMAL_ESTIMATION_THREADS);
          double radius_search = 0.025;
          if (!nh_->getParam ("/Perceptors/Visual3D/features/normals/radius_search", radius_search))
            ROS_WARN("no such parameter as </Perceptors/Visual3D/features/normals/radius_search>");
          surface_feature_extractor->getNormalEstimator ()->setRadiusSearch (radius_search);
        }
        if (f_curvatures_active_)
        {
          //TODO:
        }
        feature_extractors_.push_back (surface_feature_extractor);
      }

      if (f_spatial_active_)
      {
        SpatialFExtractor* spatial_feature_extractor = new SpatialFExtractor (nh_);
        feature_extractors_.push_back (spatial_feature_extractor);
      }

      if (f_human_active_)
      {
        HumanFExtractor* human_feature_extractor = new HumanFExtractor (nh_);
        feature_extractors_.push_back (human_feature_extractor);
      }

      //DEV_NOTE: this turns out to be necessary, due to some tf related problems, service server somehow drops the TCP connection
      //This issue needs further clarification/
      srv_cl_tabletop_detection_
          = nh_->serviceClient<tabletop_object_detector::TabletopSegmentation> (TABLETOP_SEGMENTATION_SRV_NAME, true);
      srv_cl_tabletop_detection_.waitForExistence ();
      srv_cl_tabletop_detection_
          = nh_->serviceClient<tabletop_object_detector::TabletopSegmentation> (TABLETOP_SEGMENTATION_SRV_NAME, true);

      //wait forever for connections
      collision_map_.connectionsEstablished (ros::Duration (-1.0));
*/
      is_ready_ = true;
    }

    bool
    Visual3D::isReady ()
    {
      //      return srv_cl_tabletop_detection_.exists ();
      return is_ready_;
    }

    void
    Visual3D::setClustersData (const std::vector<sensor_msgs::PointCloud>& clusters)
    {
      clusters_data_.resize (clusters.size ());
      for (uint8_t i = 0; i < clusters_data_.size (); i++)
      {
        clusters_data_[i].reset (new pcl::PointCloud<pcl::PointXYZ>);
        clusters_data_[i]->header = clusters[i].header;
        clusters_data_[i]->width = clusters[i].points.size ();
        clusters_data_[i]->height = 1;
        clusters_data_[i]->points.resize (clusters[i].points.size ());

        for (uint j = 0; j < clusters_data_[i]->points.size (); j++)
        {
          clusters_data_[i]->points[j].x = clusters[i].points[j].x;
          clusters_data_[i]->points[j].y = clusters[i].points[j].y;
          clusters_data_[i]->points[j].z = clusters[i].points[j].z;
        }
      }
    }

    void
    Visual3D::filterClusters (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters_data)
    {
      if (filter_cluster_clouds_ != NULL)
      {
        for (uint i = 0; i < clusters_data.size (); i++)
        {
          filter_cluster_clouds_->setInputCloud (clusters_data[i]);
          filter_cluster_clouds_->filter (*clusters_data[i]);
        }
      }

      for (uint i = 0; i < clusters_data.size (); i++)
      {
        //check if there is a small object after the segmentation, if so erase it
        if (clusters_data[i]->points.size () < MIN_OBJECT_CLUSTER_SIZE)
        {
          clusters_data.erase (clusters_data.begin () + i);
          --i;
        }
      }
    }
/*
    al_msgs::Entity
    Visual3D::tableToEntityMsg (const tabletop_object_detector::Table& table)
    {
      //create table and update
      arm_navigation_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = "base_footprint";
      collision_object.header.stamp = ros::Time::now ();
      collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
      arm_navigation_msgs::Shape shape;
      shape.type = arm_navigation_msgs::Shape::BOX;
      shape.dimensions.resize (3);
      shape.dimensions[0] = fabs (table.x_max - table.x_min);
      shape.dimensions[1] = fabs (table.y_max - table.y_min);
      shape.dimensions[2] = TABLE_SURFACE_THICKNESS;
      collision_object.shapes.push_back (shape);
      collision_object.id = al::facilities::toString (0);// 0 id is reserved for the table
      //DEV_NOTE: position represents the top surface of the table, be careful while boxing it!
      //this can be be solved by reading visualization markers to find corner points, it is
      //enough to find the point having smallest x and y, the rest can be handled easily.
      //      collision_object.poses.push_back (table.pose.pose);
      collision_object.poses.resize (1);
      //      collision_object.poses[0].position.x = (table.x_max + table.x_min) / 2.0;
      collision_object.poses[0].position = table.pose.pose.position;
      collision_object.poses[0].position.x += (table.x_max + table.x_min) / 2.0;
      collision_object.poses[0].position.y -= (table.y_max + table.y_min) / 2.0;
      //      collision_object.poses[0].position = table.pose.pose.position;
      //      collision_object.poses[0].position.y = (table.y_max + table.y_min) / 2.0;
      //      collision_object.poses[0].position.y = 0.0;
      //      collision_object.poses[0].position.y = table.y_min;
      //      collision_object.poses[0].position.y = table.y_max;
      collision_object.poses[0].position.z = table.pose.pose.position.z - TABLE_SURFACE_THICKNESS / 2;
      collision_object.poses[0].orientation = table.pose.pose.orientation;

      al_msgs::Entity entity;
      entity.collision_object = collision_object;

      return entity;
    }
*/
//    bool
//    Visual3D::cloudToEntitiesMsg (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data,
//                                  const tabletop_object_detector::Table& table, std::vector<al_msgs::Entity> &entities)
//    {
//      sensor_msgs::PointCloud pc_cluster = toPCMsg (cloud_data);
//      geometry_msgs::PoseStamped bbox_pose;
//      geometry_msgs::Vector3 bbox_dims;
//      collision_map_.getClusterBoundingBox (pc_cluster, bbox_pose, bbox_dims);
//      std::cout << "!!!!" << bbox_dims << std::endl;
//      std::cout << "!!!!" << bbox_pose.pose.position << std::endl;
//      std::cout << table.pose.pose.position.z << std::endl;
//
//      if (bbox_pose.pose.position.z - bbox_dims.z / 2 - table.pose.pose.position.z < BOX_EXTEND_MAX_Z_THRESHOLD)
//        collision_map_.extendBoundingBoxZToTable (table, bbox_pose, bbox_dims);
//      else
//      {
//        //this cloud is on air, don't bother identifying it, probably it belongs to non-self_filtered
//        //human related data, or robot lifted the object above
//        //TODO: this case might need further improvements
//        ROS_WARN("not on the table");
//        return false;
//      }
//
//      if (bbox_dims.x > 0.25 || bbox_dims.y > 0.25 || bbox_dims.z > 0.25)
//      {
//        ROS_WARN("not an interactable object!");
//        return false;
//      }
//      //      std::cout<<"!!!!"<<bbox_dims<<std::endl;
//      //      std::cout<<"!!!!"<<bbox_pose.pose.orientation<<std::endl;
//
//      arm_navigation_msgs::CollisionObject collision_object;
//      collision_object.header.frame_id = "base_footprint";
//      collision_object.header.stamp = ros::Time::now ();
//      collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
//      arm_navigation_msgs::Shape shape;
//      shape.type = arm_navigation_msgs::Shape::BOX;
//      shape.dimensions.resize (3);
//      shape.dimensions[0] = bbox_dims.x;
//      shape.dimensions[1] = bbox_dims.y;
//      shape.dimensions[2] = bbox_dims.z;
//      collision_object.shapes.push_back (shape);
//      collision_object.shapes[0].dimensions.resize (3);
//      collision_object.poses.push_back (bbox_pose.pose);
//
//      sensor_msgs::PointCloud2 pc2_cluster;
//      pcl::toROSMsg (*cloud_data, pc2_cluster);
//      al_msgs::Entity entity;
//      entity.cloud_object = pc2_cluster;
//
//      //identification of the object
//
//      //if it intersects with a prev_collision_object more than 75% of the volume of that object
//      //it is most probably that his object is that object, or this is a part of that object now.
//
//      bool added_itself = false;
//      //      std::cout << "#prev_obj: " << prev_collision_objects_.collision_objects.size () << ::std::endl;
//      for (uint8_t i = 0; i < prev_collision_objects_.collision_objects.size (); i++)
//      {
//        //discard intersection test with the table, which is not possible in anyway with rigid objects
//        if (!al::facilities::toInt (prev_collision_objects_.collision_objects[i].id))
//          continue;
//
//        //        std::cout << "*1" << std::endl;
//        //        std::cout << collision_object.poses[0].position << std::endl;
//        //        std::cout << collision_object.shapes[0].dimensions[0] << " " << collision_object.shapes[0].dimensions[1] << " "
//        //            << collision_object.shapes[0].dimensions[2] << "\n";
//
//        float intersection_vol = al::math::getIntersectionVolume (collision_object,
//                                                                  prev_collision_objects_.collision_objects[i]);
//        float obj_vol = al::math::getVolume (collision_object);
//        float prev_obj_vol = al::math::getVolume (prev_collision_objects_.collision_objects[i]);
//        //        std::cout << "obj_vol: " << obj_vol << "\nprev_obj_vol:" << prev_obj_vol << "\nint_objs_vol:"
//        //            << intersection_vol << std::endl;
//
//        //if the intersection volume and the shape is similar to this box's volume, then this is itself.
//        //if this intersection volume and the shape is similar to prev_obj volume and shape,
//        //then the id is this
//        if (intersection_vol > obj_vol * BOX_SIMILARITY_LOWER_VOL_THRESH && intersection_vol < obj_vol
//            * BOX_SIMILARITY_UPPER_VOL_THRESH)
//        {
//          if (!added_itself)
//          {
//            geometry_msgs::Vector3 prev_object_bbox_dims;
//            prev_object_bbox_dims.x = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[0];
//            prev_object_bbox_dims.y = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[1];
//            prev_object_bbox_dims.z = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[2];
//
//            double difference = al::math::calBoxSimilarity (prev_object_bbox_dims, bbox_dims);
//
//            //be more tolerable during shape similarity check
//            if (difference < BOX_SIMILARITY_THRESHOLD * 1.5)
//            {
//              //              std::cout << "*3" << std::endl;
//              //found the object id -- it is the same object -- no merging -- no end-effector intervention
//              added_itself = true;
//              entity.collision_object = collision_object;
//              entity.collision_object.id = prev_collision_objects_.collision_objects[i].id;
//              entities.push_back (entity);
//            }
//            else
//            {
//              ROS_WARN("found intersecting volume with a prev_obj, but the shape differs, this is strange!");
//            }
//          }
//          else
//          {
//            ROS_WARN("found intersecting volume with a prev_obj, but it already added itself, if this is not itself, what can it be? This is strange!");
//          }
//        }///*
//        //if intersection volume looks like other object
//        else if (intersection_vol > prev_obj_vol * BOX_SIMILARITY_LOWER_VOL_THRESH / 3.0 && intersection_vol
//            < prev_obj_vol * BOX_SIMILARITY_UPPER_VOL_THRESH)
//        {
//          //this should belong to an object that is contained by this box, they shouldn't look like each other in shape
//          //          std::cout << "*2" << std::endl;
//          geometry_msgs::Vector3 prev_object_bbox_dims;
//          prev_object_bbox_dims.x = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[0];
//          prev_object_bbox_dims.y = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[1];
//          prev_object_bbox_dims.z = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[2];
//
//          double difference = al::math::calBoxSimilarity (prev_object_bbox_dims, bbox_dims);
//
//          //be more tolerable during shape similarity check
//          if (difference < BOX_SIMILARITY_THRESHOLD * 1.5)
//          {
//            ROS_WARN ("intersection doesn't look like box itself, but prev_object. But the box and prev_object are similar in shape. This shouldn't occur!");
//          }
//          else
//          {
//            //            std::cout << "*4" << std::endl;
//            entity.collision_object = prev_collision_objects_.collision_objects[i];
//            entities.push_back (entity);
//          }
//        }
//        else
//        {
//          if (intersection_vol > 0)//there is an intersection but looks no one more than 75%
//            ROS_WARN("this setup cannot support this kind of intersection, there should be something wrong!");
//        }
//        //*/
//      }
//      entity.collision_object = collision_object;
//      //still couldn't find the id of the object (from prev_objects)
//      if (!added_itself)
//      {
//        //        std::cout << "*5" << std::endl;
//        //check if there is a similar box in the environment.
//        //it might be a case that an object moved too fast w.r.t to its size, and intersecting volume becomes
//        //less than the similarity threshold. It is assumed that there is no object or object combination
//        //in the environment that might yield wrong identification (e.g. same id)
//        if (identifyEntityFromPrevIds (entity))
//        {
//          added_itself = true;
//          entities.push_back (entity);
//          return true;
//        }
//      }
//
//      //still couldn't find the id of the object from what is seen in the environment,
//      //therefore this object is a newcomer (might have been seen earlier though)
//
//      //get most similar identified object id, no need to be as tolerable as the previous case
//      if (!added_itself)
//      {
//        identifyEntityFromKnownIds (entity);
//        entities.push_back (entity);
//        return true;
//      }
//      if (added_itself)
//        return true;
//      else
//        return false;//this shouldn't happen
//    }

    bool
    Visual3D::identifyEntityFromPrevIds (al_msgs::Entity& entity)
    {
      geometry_msgs::Vector3 bbox_dims;
      bbox_dims.x = entity.collision_object.shapes[0].dimensions[0];
      bbox_dims.y = entity.collision_object.shapes[0].dimensions[1];
      bbox_dims.z = entity.collision_object.shapes[0].dimensions[2];

      //get most similar prev object id, no need to be as tolerable as the previous case
      double min_difference = DBL_MAX;
      int most_similar_id = -1;
      for (uint8_t i = 1; i < prev_collision_objects_.collision_objects.size (); i++)
      {
        //        std::cout << "*6" << std::endl;
        geometry_msgs::Vector3 prev_object_bbox_dims;
        prev_object_bbox_dims.x = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[0];
        prev_object_bbox_dims.y = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[1];
        prev_object_bbox_dims.z = prev_collision_objects_.collision_objects[i].shapes[0].dimensions[2];

        double difference = al::math::calBoxSimilarity (prev_object_bbox_dims, bbox_dims);
        if (difference < min_difference)
        {
          //          std::cout << "*7" << std::endl;
          min_difference = difference;
          most_similar_id = i;
        }
      }

      if (min_difference < BOX_SIMILARITY_THRESHOLD)
      {
        //        std::cout << "*8" << std::endl;
        entity.collision_object.id = prev_collision_objects_.collision_objects[most_similar_id].id;
        return true;
      }
      return false;
    }

    bool
    Visual3D::identifyEntityFromKnownIds (al_msgs::Entity& entity)
    {
      geometry_msgs::Vector3 bbox_dims;
      bbox_dims.x = entity.collision_object.shapes[0].dimensions[0];
      bbox_dims.y = entity.collision_object.shapes[0].dimensions[1];
      bbox_dims.z = entity.collision_object.shapes[0].dimensions[2];

      //      if (map_identified_box_dims_.size () > 10)
      //        map_identified_box_dims_.clear ();

      //get most similar identified object id, no need to be as tolerable as the previous case
      double min_difference = DBL_MAX;
      int most_similar_id = -1;
      std::map<int, geometry_msgs::Vector3>::iterator it;
      for (it = map_identified_box_dims_.begin (); it != map_identified_box_dims_.end (); it++)
      {
        //        std::cout << "*9" << std::endl;
        double difference = al::math::calBoxSimilarity ((*it).second, bbox_dims);
        if (difference < min_difference)
        {
          //          std::cout << "*10" << std::endl;
          min_difference = difference;
          most_similar_id = (*it).first;
        }
      }

      if (min_difference < BOX_SIMILARITY_THRESHOLD)
      {
        //        std::cout << "*11" << std::endl;
        //we got the id of a similar object, assign this
        ROS_DEBUG("this id should be positive %d", most_similar_id);
        entity.collision_object.id = al::facilities::toString (most_similar_id);
        return true;
      }
      else
      {
        //        std::cout << "*12" << std::endl;
        //assign new id
        map_identified_box_dims_.insert (
                                         map_identified_box_dims_.end (),
                                         std::pair<int, geometry_msgs::Vector3> (
                                                                                 ((int)map_identified_box_dims_.end ()->first
                                                                                     + 1), bbox_dims));
        entity.collision_object.id = al::facilities::toString ((int)(*map_identified_box_dims_.end ()).first);
        return false;
      }
    }

//    void
//    Visual3D::extractEntities (const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters_data,
//                               const tabletop_object_detector::Table& table)
//    {
//      entities_.entities.clear ();
//      entities_.header.stamp = ros::Time::now ();
//
//      //get table as entity
//      entities_.entities.push_back (tableToEntityMsg (table));
//
//      //get objects as entities
//      std::cout << "#######clusters:" << (int)clusters_data.size () << std::endl;
//      for (uint8_t i = 0; i < clusters_data.size (); i++)
//      {
//        std::vector<al_msgs::Entity> entities;
//        if (cloudToEntitiesMsg (clusters_data[i], table, entities))
//        {
//          //          std::cout << "cluster " << (int)(i + 1) << "  #entities_added: " << entities.size () << std::endl;
//          entities_.entities.insert (entities_.entities.end (), entities.begin (), entities.end ());
//        }
//      }
//      //      std::cout << "#entities: " << (int)entities_.entities.size () << std::endl;
//      //      for (uint i = 0; i < entities_.entities.size (); i++)
//      //        std::cout << "entity id: " << entities_.entities[i].collision_object.id << std::endl;
//    }

    int
    Visual3D::percept ()
    {
      if (pointcloud_rcvd_)
      {
        bool call_segmentation_srv_again = true;
        int n_service_call = 0;
        //do a persistent service call for N_PERSISTENT_SERVICE_CALL times. Note that this may block the whole experiment.
        while (nh_->ok () && call_segmentation_srv_again && n_service_call < N_PERSISTENT_SERVICE_CALL)
        {
          //call the tabletop detection service
          //ROS_INFO("Calling tabletop segmentator");

//          if (!srv_cl_tabletop_detection_.call (tabletop_segmentation_))//service couldn't be called, quit
//
//          {
//            ROS_ERROR("Tabletop detection service failed");
//            return -1;
//          }
//          else//service successfully called
//
//          {
//            n_service_call++;
//
//            if (tabletop_segmentation_.response.result == tabletop_segmentation_.response.NO_CLOUD_RECEIVED)
//            {
//              ROS_WARN("no point cloud received by segmentation service. Service is being recalled now...");
//            }
//            else if (tabletop_segmentation_.response.result == tabletop_segmentation_.response.NO_TABLE)
//            {
//              ROS_WARN("Something wrong with the table (no-table). Service is being recalled now...");
//            }
//            else if (tabletop_segmentation_.response.result == tabletop_segmentation_.response.OTHER_ERROR)//re-send point cloud
//
//            {
//              ROS_WARN("Possible transformation (tf) problem. Service is being recalled now...");
//            }
//            else if (tabletop_segmentation_.response.result == tabletop_segmentation_.response.SUCCESS)
//            {
//              ROS_INFO("Successful segmentation!");
//              call_segmentation_srv_again = false;
//            }
//          }
          ros::spinOnce ();
        }

        //TODO: segmentation wasn't successful, handle this case in the experimenter module
        if (call_segmentation_srv_again)
          return -2;
      }
      else
        return -1;

//      setClustersData (tabletop_segmentation_.response.clusters);
//      filterClusters (clusters_data_);
//      extractEntities (clusters_data_, tabletop_segmentation_.response.table);
      publishEntities ();
      publishCollisionObjects ();
      //      publishCloudObjects ();

      return 1;
    }

    bool
    Visual3D::srvGetEntityVisualFeatures (al_srvs::GetEntityVisualFeatures::Request &req,
                                          al_srvs::GetEntityVisualFeatures::Response &res)
    {
      ROS_DEBUG("#feature_extractors: %d", (int)feature_extractors_.size());
      al::perception::Entity entity = al::perception::fromEntityMsg (req.entity);

      for (uint8_t i = 0; i < feature_extractors_.size (); i++)
        feature_extractors_[i]->extract (entity);

      res.feature_vector = entity.getFeatures ();
      return true;
    }

    bool
    Visual3D::srvGetEntitiesVisualFeatures (al_srvs::GetEntitiesVisualFeatures::Request &req,
                                            al_srvs::GetEntitiesVisualFeatures::Response &res)
    {
      ROS_DEBUG("#feature_extractors: %d", (int)feature_extractors_.size());
      std::cout << "called" << std::endl;

      std::vector<al::perception::Entity> entities (req.entities.entities.size ());
      for (uint8_t i = 0; i < entities.size (); i++)
        entities[i] = al::perception::fromEntityMsg (req.entities.entities[i]);

      for (uint8_t i = 0; i < entities.size (); i++)
        for (uint8_t j = 0; j < feature_extractors_.size (); j++)
          feature_extractors_[j]->extract (entities[i]);

      res.feature_vectors.resize (entities.size ());
      for (uint8_t i = 0; i < res.feature_vectors.size (); i++)
        res.feature_vectors[i] = entities[i].getFeatures ();

      return true;
    }

    void
    Visual3D::publishEntities ()
    {
      if (pub_entities_.getNumSubscribers ())
        pub_entities_.publish (entities_);
    }

    void
    Visual3D::publishCollisionObjects ()
    {
      collision_objects_.collision_objects.resize (entities_.entities.size ());
      for (uint i = 0; i < entities_.entities.size (); i++)
        collision_objects_.collision_objects[i] = entities_.entities[i].collision_object;

//      if (pub_collision_objects_.getNumSubscribers ())
//        pub_collision_objects_.publish (collision_objects_);
//
//      if (pub_collision_object_.getNumSubscribers ())
//      {
//        for (uint i = 0; i < collision_objects_.collision_objects.size (); i++)
//          pub_collision_object_.publish (collision_objects_.collision_objects[i]);
//      }

      //now remove the objects which do not exist in the last perception cycle
      std::vector<bool> non_used_prev_objects (prev_collision_objects_.collision_objects.size (), true);
      for (uint i = 0; i < prev_collision_objects_.collision_objects.size (); i++)
      {
        for (uint j = 0; j < collision_objects_.collision_objects.size (); j++)
        {
          if (prev_collision_objects_.collision_objects[i].id == collision_objects_.collision_objects[j].id)
          {
            non_used_prev_objects[i] = false;
            break;
          }
        }
      }
      for (uint i = 0; i < prev_collision_objects_.collision_objects.size (); i++)
      {
        if (non_used_prev_objects[i])
        {
          prev_collision_objects_.collision_objects[i].operation.operation
              = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
          pub_collision_object_.publish (prev_collision_objects_.collision_objects[i]);
        }
      }

      prev_collision_objects_ = collision_objects_;
    }

    //    void
    //    Visual3D::publishCloudObjects ()
    //    {
    //      cloud_objects_.cloud_objects.resize (entities_.entities.size ());
    //      for (uint i = 0; i < entities_.entities.size (); i++)
    //        cloud_objects_.cloud_objects[i] = entities_.entities[i].cloud_object;
    //
    //      if (pub_cloud_objects_.getNumSubscribers ())
    //        pub_cloud_objects_.publish (cloud_objects_);
    //    }

    void
    Visual3D::pc2RcvdCallback (sensor_msgs::PointCloud2::ConstPtr pc2_msg)
    {
      if (!pointcloud_rcvd_)
        pointcloud_rcvd_ = true;

      pcl::fromROSMsg (*pc2_msg, *pointcloud_data_);
      ros::Time t1 = ros::Time::now ();
      for (uint8_t i = 0; i < filters_cloud_.size (); i++)
      {
        filters_cloud_[i]->setInputCloud (pointcloud_data_);
        filters_cloud_[i]->filter (*pointcloud_data_);
      }
      ROS_DEBUG("application of all filters take: %f seconds ", (ros::Time::now()-t1).toSec() );

      pcl::toROSMsg (*pointcloud_data_, msg_pointcloud_filtered_);

      //first filter the cloud
      if (pub_pointcloud_filtered_.getNumSubscribers () > 0)
      {
        ROS_DEBUG("msg_pointcloud_filtered_ has been sent");
        pub_pointcloud_filtered_.publish (msg_pointcloud_filtered_);
      }
    }

    Visual3D::~Visual3D ()
    {
    }
  }
}
