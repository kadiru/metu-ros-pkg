/*********************************************************************
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author(s): Marius Muja and Matei Ciocarlie
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl16/point_cloud.h>
#include <pcl16/point_types.h>
#include <pcl16/io/io.h>
#include <pcl16/filters/voxel_grid.h>
#include <pcl16/filters/passthrough.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/features/normal_3d.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/filters/project_inliers.h>
#include <pcl16/surface/convex_hull.h>
#include <pcl16/search/kdtree.h>
#include <pcl16/segmentation/extract_polygonal_prism_data.h>
#include <pcl16/segmentation/extract_clusters.h>
//#include <pcl16_ros/transforms.h>
#include "pcl16_ros/impl/transforms.hpp"
#include "pcl16/filters/impl/passthrough.hpp"
#include "pcl16/filters/impl/voxel_grid.hpp"

#include "tabletop_object_segmentation/marker_generator.h"
#include "tabletop_object_segmentation/TabletopSegmentation.h"

const int MAX_N_CLUSTERS = 5;

namespace tabletop_object_segmentation
{

  class TabletopSegmentor
  {
    typedef pcl16::PointXYZRGB Point;
    typedef pcl16::search::KdTree<Point>::Ptr KdTreePtr;

  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    //! Publisher for markers
    ros::Publisher marker_pub_;
    //! Service server for object detection
    ros::ServiceServer segmentation_srv_;

    //++Kadir
    std::vector<ros::Publisher> pubs_clusters_;
//    ros::Publisher pub_debug_tabletop;
    //--Kadir

    //! Used to remember the number of markers we publish so we can delete them later
    int num_markers_published_;
    //! The current marker being published
    int current_marker_id_;

    //! Min number of inliers for reliable plane detection
    int inlier_threshold_;
    //! Size of downsampling grid before performing plane detection
    double plane_detection_voxel_size_;
    //! Size of downsampling grid before performing clustering
    double clustering_voxel_size_;
    //! Filtering of original point cloud along the z, y, and x axes
    double z_filter_min_, z_filter_max_;
    double y_filter_min_, y_filter_max_;
    double x_filter_min_, x_filter_max_;
    //! Filtering of point cloud in table frame after table detection
    double table_z_filter_min_, table_z_filter_max_;
    //! Min distance between two clusters
    double cluster_distance_;
    //! Min number of points for a cluster
    int min_cluster_size_;
    //! Clouds are transformed into this frame before processing; leave empty if clouds
    //! are to be processed in their original frame
    std::string processing_frame_;
    //! Positive or negative z is closer to the "up" direction in the processing frame?
    double up_direction_;
    bool flatten_table_;
    //! How much the table gets padded in the horizontal direction
    double table_padding_;

    //! A tf transform listener
    tf::TransformListener listener_;

    //------------------ Callbacks -------------------

    //! Callback for service calls
    bool
    serviceCallback (TabletopSegmentation::Request &request, TabletopSegmentation::Response &response);

    //------------------ Individual processing steps -------

    //! Converts raw table detection results into a Table message type
    template<class PointCloudType>
      Table
      getTable (std_msgs::Header cloud_header, const tf::Transform &table_plane_trans,
                const PointCloudType &table_points);

    //! Converts table convex hull into a triangle mesh to add to a Table message
    template<class PointCloudType>
      void
      addConvexHullTable (Table &table, const PointCloudType &convex_hull, bool flatten_table);

    //! Publishes rviz markers for the given tabletop clusters
    template<class PointCloudType>
      void
      publishClusterMarkers (const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);

    //------------------- Complete processing -----

    //! Complete processing for new style point cloud
    void
    processCloud (const sensor_msgs::PointCloud2 &cloud, TabletopSegmentation::Response &response, Table table);

    //! Clears old published markers and remembers the current number of published markers
    void
    clearOldMarkers (std::string frame_id);

    //! Pull out and transform the convex hull points from a Table message
    template<class PointCloudType>
      bool
      tableMsgToPointCloud (Table &table, std::string frame_id, PointCloudType &table_hull);

  public:
    //! Subscribes to and advertises topics; initializes fitter and marker publication flags
    /*! Also attempts to connect to database */
    TabletopSegmentor (ros::NodeHandle nh) :
        nh_ (nh), priv_nh_ ("~")
    {
      num_markers_published_ = 1;
      current_marker_id_ = 1;

      marker_pub_ = nh_.advertise<visualization_msgs::Marker> (nh_.resolveName ("markers_out"), 10);

      //++Kadir
      pubs_clusters_.resize (MAX_N_CLUSTERS);
      for (uint i = 0; i < pubs_clusters_.size (); i++)
      {
        std::stringstream s;
        s << (int)i;
        pubs_clusters_[i] = nh_.advertise<sensor_msgs::PointCloud2> ("debug_cluster_" + s.str (), 10);
      }
//        pub_debug_tabletop = nh_.advertise<sensor_msgs::PointCloud2> ("debug_tabletop", 10);
      //--Kadir

      segmentation_srv_ = nh_.advertiseService (nh_.resolveName ("segmentation_srv"),
                                                &TabletopSegmentor::serviceCallback, this);

      //initialize operational flags
      priv_nh_.param<int> ("inlier_threshold", inlier_threshold_, 300);
      priv_nh_.param<double> ("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
      priv_nh_.param<double> ("clustering_voxel_size", clustering_voxel_size_, 0.003);
      priv_nh_.param<double> ("z_filter_min", z_filter_min_, 0.4);
      priv_nh_.param<double> ("z_filter_max", z_filter_max_, 1.25);
      priv_nh_.param<double> ("y_filter_min", y_filter_min_, -1.0);
      priv_nh_.param<double> ("y_filter_max", y_filter_max_, 1.0);
      priv_nh_.param<double> ("x_filter_min", x_filter_min_, -1.0);
      priv_nh_.param<double> ("x_filter_max", x_filter_max_, 1.0);
      priv_nh_.param<double> ("table_z_filter_min", table_z_filter_min_, 0.01);
      priv_nh_.param<double> ("table_z_filter_max", table_z_filter_max_, 0.50);
      priv_nh_.param<double> ("cluster_distance", cluster_distance_, 0.03);
      priv_nh_.param<int> ("min_cluster_size", min_cluster_size_, 300);
      priv_nh_.param<std::string> ("processing_frame", processing_frame_, "");
      priv_nh_.param<double> ("up_direction", up_direction_, -1.0);
      priv_nh_.param<bool> ("flatten_table", flatten_table_, false);
      priv_nh_.param<double> ("table_padding", table_padding_, 0.0);
      if (flatten_table_)
        ROS_DEBUG("flatten_table is true");
      else
        ROS_DEBUG("flatten_table is false");

    }

    //! Empty stub
    ~TabletopSegmentor ()
    {
    }
  };

  /*! Processes the latest point cloud and gives back the resulting array of models.
   */
  bool
  TabletopSegmentor::serviceCallback (TabletopSegmentation::Request &request, TabletopSegmentation::Response &response)
  {
    ros::Time start_time = ros::Time::now ();
    std::string topic = nh_.resolveName ("cloud_in");
    ROS_INFO( "Tabletop detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());

    sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2> (
        topic, nh_, ros::Duration (3.0));

    if (!recent_cloud)
    {
      ROS_ERROR( "Tabletop object detector: no point_cloud2 has been received");
      response.result = response.NO_CLOUD_RECEIVED;
      return true;
    }

    pcl16::PointCloud<Point>::Ptr table_hull (new pcl16::PointCloud<Point>);
    ROS_INFO_STREAM( "Point cloud received after " << ros::Time::now() - start_time << " seconds; processing");
    if (!processing_frame_.empty ())
    {
      //convert cloud to processing_frame_ (usually base_link)
      sensor_msgs::PointCloud old_cloud;
      sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
      int current_try = 0, max_tries = 3;
      while (1)
      {
        bool transform_success = true;
        try
        {
          listener_.transformPointCloud (processing_frame_, old_cloud, old_cloud);
        }
        catch (tf::TransformException ex)
        {
          transform_success = false;
          if (++current_try >= max_tries)
          {
            ROS_ERROR(
                "Failed to transform cloud from frame %s into frame %s in %d attempt(s)", old_cloud.header.frame_id.c_str(), processing_frame_.c_str(), current_try);
            response.result = response.OTHER_ERROR;
            return true;
          }
          ROS_DEBUG(
              "Failed to transform point cloud, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());
          //sleep a bit to give the listener a chance to get a new transform
          ros::Duration (0.1).sleep ();
        }
        if (transform_success)
          break;
      }
      sensor_msgs::PointCloud2 converted_cloud;
      sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
      ROS_INFO_STREAM(
          "Input cloud converted to " << processing_frame_ << " frame after " << ros::Time::now() - start_time << " seconds");
      processCloud (converted_cloud, response, request.table);
      clearOldMarkers (converted_cloud.header.frame_id);
    }
    else
    {
      processCloud (*recent_cloud, response, request.table);
      clearOldMarkers (recent_cloud->header.frame_id);
    }

    //add the timestamp from the original cloud
    response.table.pose.header.stamp = recent_cloud->header.stamp;
    for (size_t i; i < response.clusters.size (); i++)
    {
      response.clusters[i].header.stamp = recent_cloud->header.stamp;
    }

    ROS_INFO_STREAM( "In total, segmentation took " << ros::Time::now() - start_time << " seconds");
    return true;
  }

  template<class PointCloudType>
    void
    TabletopSegmentor::addConvexHullTable (Table &table, const PointCloudType &convex_hull, bool flatten_table)
    {
      if (convex_hull.points.empty ())
      {
        ROS_ERROR("Trying to add convex hull, but it contains no points");
        return;
      }
      //compute centroid
      geometry_msgs::Point centroid;
      centroid.x = centroid.y = centroid.z = 0.0;
      for (size_t i = 0; i < convex_hull.points.size (); i++)
      {
        centroid.x += convex_hull.points[i].x;
        centroid.y += convex_hull.points[i].y;
        centroid.z += convex_hull.points[i].z;
      }
      centroid.x /= convex_hull.points.size ();
      centroid.y /= convex_hull.points.size ();
      centroid.z /= convex_hull.points.size ();

      //create a triangle mesh out of the convex hull points and add it to the table message
      table.convex_hull.type = table.convex_hull.MESH;
      for (size_t i = 0; i < convex_hull.points.size (); i++)
      {
        geometry_msgs::Point vertex;
        vertex.x = convex_hull.points[i].x;
        vertex.y = convex_hull.points[i].y;
        if (table_padding_ > 0.0)
        {
          double dx = vertex.x - centroid.x;
          double dy = vertex.y - centroid.y;
          double l = sqrt (dx * dx + dy * dy);
          dx /= l;
          dy /= l;
          vertex.x += table_padding_ * dx;
          vertex.y += table_padding_ * dy;
        }
        if (flatten_table)
          vertex.z = 0;
        else
          vertex.z = convex_hull.points[i].z;
        table.convex_hull.vertices.push_back (vertex);

        if (i == 0 || i == convex_hull.points.size () - 1)
          continue;
        table.convex_hull.triangles.push_back (0);
        table.convex_hull.triangles.push_back (i);
        table.convex_hull.triangles.push_back (i + 1);
      }
      visualization_msgs::Marker tableMarker = MarkerGenerator::getConvexHullTableMarker (table.convex_hull);
      tableMarker.header = table.pose.header;
      tableMarker.pose = table.pose.pose;
      tableMarker.ns = "tabletop_node";
      tableMarker.id = current_marker_id_++;
      marker_pub_.publish (tableMarker);

      visualization_msgs::Marker originMarker = MarkerGenerator::createMarker (table.pose.header.frame_id, 0, .0025,
                                                                               .0025, .01, 0, 1, 1,
                                                                               visualization_msgs::Marker::CUBE,
                                                                               current_marker_id_++, "tabletop_node",
                                                                               table.pose.pose);
      marker_pub_.publish (originMarker);
    }

  template<class PointCloudType>
    Table
    TabletopSegmentor::getTable (std_msgs::Header cloud_header, const tf::Transform &table_plane_trans,
                                 const PointCloudType &table_points)
    {
      Table table;

      //get the extents of the table
      if (!table_points.points.empty ())
      {
        table.x_min = table_points.points[0].x;
        table.x_max = table_points.points[0].x;
        table.y_min = table_points.points[0].y;
        table.y_max = table_points.points[0].y;
      }
      for (size_t i = 1; i < table_points.points.size (); ++i)
      {
        if (table_points.points[i].x < table.x_min && table_points.points[i].x > -3.0)
          table.x_min = table_points.points[i].x;
        if (table_points.points[i].x > table.x_max && table_points.points[i].x < 3.0)
          table.x_max = table_points.points[i].x;
        if (table_points.points[i].y < table.y_min && table_points.points[i].y > -3.0)
          table.y_min = table_points.points[i].y;
        if (table_points.points[i].y > table.y_max && table_points.points[i].y < 3.0)
          table.y_max = table_points.points[i].y;
      }

      geometry_msgs::Pose table_pose;
      tf::poseTFToMsg (table_plane_trans, table_pose);
      table.pose.pose = table_pose;
      table.pose.header = cloud_header;

      visualization_msgs::Marker tableMarker = MarkerGenerator::getTableMarker (table.x_min, table.x_max, table.y_min,
                                                                                table.y_max);
      tableMarker.header = cloud_header;
      tableMarker.pose = table_pose;
      tableMarker.ns = "tabletop_node";
      tableMarker.id = current_marker_id_++;
      marker_pub_.publish (tableMarker);

      return table;
    }

  template<class PointCloudType>
    void
    TabletopSegmentor::publishClusterMarkers (const std::vector<PointCloudType> &clusters,
                                              std_msgs::Header cloud_header)
    {
      for (size_t i = 0; i < clusters.size (); i++)
      {
        visualization_msgs::Marker cloud_marker = MarkerGenerator::getCloudMarker (clusters[i]);
        cloud_marker.header = cloud_header;
        cloud_marker.pose.orientation.w = 1;
        cloud_marker.ns = "tabletop_node";
        cloud_marker.id = current_marker_id_++;
        marker_pub_.publish (cloud_marker);
      }
    }

  void
  TabletopSegmentor::clearOldMarkers (std::string frame_id)
  {
    for (int id = current_marker_id_; id < num_markers_published_; id++)
    {
      visualization_msgs::Marker delete_marker;
      delete_marker.header.stamp = ros::Time::now ();
      delete_marker.header.frame_id = frame_id;
      delete_marker.id = id;
      delete_marker.action = visualization_msgs::Marker::DELETE;
      delete_marker.ns = "tabletop_node";
      marker_pub_.publish (delete_marker);
    }
    num_markers_published_ = current_marker_id_;
    current_marker_id_ = 0;
  }

  /*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
  tf::Transform
  getPlaneTransform (pcl16::ModelCoefficients coeffs, double up_direction, bool flatten_plane)
  {
    ROS_ASSERT(coeffs.values.size() > 3);
    double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
    //asume plane coefficients are normalized
    tf::Vector3 position (-a * d, -b * d, -c * d);
    tf::Vector3 z (a, b, c);

    //if we are flattening the plane, make z just be (0,0,up_direction)
    if (flatten_plane)
    {
      ROS_INFO("flattening plane");
      z[0] = z[1] = 0;
      z[2] = up_direction;
    }
    else
    {
      //make sure z points "up"
      ROS_DEBUG( "in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
      if (z.dot (tf::Vector3 (0, 0, up_direction)) < 0)
      {
        z = -1.0 * z;
        ROS_INFO("flipped z");
      }
    }

    //try to align the x axis with the x axis of the original frame
    //or the y axis if z and x are too close too each other
    tf::Vector3 x (1, 0, 0);
    if (fabs (z.dot (x)) > 1.0 - 1.0e-4)
      x = tf::Vector3 (0, 1, 0);
    tf::Vector3 y = z.cross (x).normalized ();
    x = y.cross (z).normalized ();

    tf::Matrix3x3 rotation;
    rotation[0] = x; 	// x
    rotation[1] = y; 	// y
    rotation[2] = z; 	// z
    rotation = rotation.transpose ();
    tf::Quaternion orientation;
    rotation.getRotation (orientation);
    ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
    ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    return tf::Transform (orientation, position);
  }

  template<class PointCloudType>
    bool
    TabletopSegmentor::tableMsgToPointCloud (Table &table, std::string frame_id, PointCloudType &table_hull)
    {
      //use table.pose (PoseStamped) to transform table.convex_hull.vertices (Point[]) into a pcl16::PointCloud in frame_id
      ros::Time now = ros::Time::now ();
      PointCloudType table_frame_points;
      table_frame_points.header.stamp = now;
      table_frame_points.header.frame_id = "table_frame";
      table_frame_points.points.resize (table.convex_hull.vertices.size ());
      for (size_t i = 0; i < table.convex_hull.vertices.size (); i++)
      {
        table_frame_points.points[i].x = table.convex_hull.vertices[i].x;
        table_frame_points.points[i].y = table.convex_hull.vertices[i].y;
        table_frame_points.points[i].z = table.convex_hull.vertices[i].z;
      }

      //add the table frame transform to the tf listener
      tf::Transform table_trans;
      tf::poseMsgToTF (table.pose.pose, table_trans);
      tf::StampedTransform table_pose_frame (table_trans, now, table.pose.header.frame_id, "table_frame");
      listener_.setTransform (table_pose_frame);
      ROS_INFO( "transforming point cloud from table frame to frame %s", frame_id.c_str());

      //make sure we can transform
      std::string error_msg;
      if (!listener_.waitForTransform (frame_id, "table_frame", now, ros::Duration (2.0), ros::Duration (0.01),
                                       &error_msg))
      {
        ROS_ERROR(
            "Can not transform point cloud from table frame to frame %s; error %s", frame_id.c_str(), error_msg.c_str());
        return false;
      }

      //transform the points
      int current_try = 0, max_tries = 3;
      while (1)
      {
        bool transform_success = true;
        try
        {
          pcl16_ros::transformPointCloud<Point> (frame_id, table_frame_points, table_hull, listener_);
        }
        catch (tf::TransformException ex)
        {
          transform_success = false;
          if (++current_try >= max_tries)
          {
            ROS_ERROR(
                "Failed to transform point cloud from table frame into frame %s; error %s", frame_id.c_str(), ex.what());
            return false;
          }
          //sleep a bit to give the listener a chance to get a new transform
          ros::Duration (0.1).sleep ();
        }
        if (transform_success)
          break;
      }
      table_hull.header.frame_id = frame_id;
      table_hull.header.stamp = now;

      //copy the transformed points back into the Table message and set the pose to identity in the cloud frame
      for (size_t i = 0; i < table.convex_hull.vertices.size (); i++)
      {
        table.convex_hull.vertices[i].x = table_hull.points[i].x;
        table.convex_hull.vertices[i].y = table_hull.points[i].y;
        table.convex_hull.vertices[i].z = table_hull.points[i].z;
      }
      geometry_msgs::PoseStamped iden;
      iden.pose.orientation.w = 1;
      table.pose = iden;
      table.pose.header.frame_id = frame_id;

      //make a new Shape for drawing
      arm_navigation_msgs::Shape mesh;
      mesh.vertices.resize (table_hull.points.size ());
      for (size_t i = 0; i < table_hull.points.size (); i++)
      {
        mesh.vertices[i].x = table_hull.points[i].x;
        mesh.vertices[i].y = table_hull.points[i].y;
        mesh.vertices[i].z = table_hull.points[i].z;
      }
      mesh.triangles = table.convex_hull.triangles;
      visualization_msgs::Marker tableMarker = MarkerGenerator::getConvexHullTableMarker (mesh);
      tableMarker.header = table_hull.header;
      tableMarker.pose.orientation.w = 1.0;
      tableMarker.ns = "tabletop_node";
      tableMarker.id = current_marker_id_++;
      marker_pub_.publish (tableMarker);

      return true;
    }

  template<typename PointT>
    bool
    getPlanePoints (const pcl16::PointCloud<PointT> &table, const tf::Transform& table_plane_trans,
                    sensor_msgs::PointCloud &table_points)
    {
      // Prepare the output
      table_points.header = table.header;
      table_points.points.resize (table.points.size ());
      for (size_t i = 0; i < table.points.size (); ++i)
      {
        table_points.points[i].x = table.points[i].x;
        table_points.points[i].y = table.points[i].y;
        table_points.points[i].z = table.points[i].z;
      }

      // Transform the data
      tf::TransformListener listener;
      tf::StampedTransform table_pose_frame (table_plane_trans, table.header.stamp, table.header.frame_id,
                                             "table_frame");
      listener.setTransform (table_pose_frame);
      std::string error_msg;
      if (!listener.canTransform ("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
      {
        ROS_ERROR(
            "Can not transform point cloud from frame %s to table frame; error %s", table_points.header.frame_id.c_str(), error_msg.c_str());
        return false;
      }
      int current_try = 0, max_tries = 3;
      while (1)
      {
        bool transform_success = true;
        try
        {
          listener.transformPointCloud ("table_frame", table_points, table_points);
        }
        catch (tf::TransformException ex)
        {
          transform_success = false;
          if (++current_try >= max_tries)
          {
            ROS_ERROR(
                "Failed to transform point cloud from frame %s into table_frame; error %s", table_points.header.frame_id.c_str(), ex.what());
            return false;
          }
          //sleep a bit to give the listener a chance to get a new transform
          ros::Duration (0.1).sleep ();
        }
        if (transform_success)
          break;
      }
      table_points.header.stamp = table.header.stamp;
      table_points.header.frame_id = "table_frame";
      return true;
    }

  template<class PointCloudType>
    void
    straightenPoints (PointCloudType &points, const tf::Transform& table_plane_trans,
                      const tf::Transform& table_plane_trans_flat)
    {
      tf::Transform trans = table_plane_trans_flat * table_plane_trans.inverse ();
      pcl16_ros::transformPointCloud (points, points, trans);
    }

  template<typename PointT>
    void
    getClustersFromPointCloud2 (const pcl16::PointCloud<PointT> &cloud_objects,
                                const std::vector<pcl16::PointIndices> &clusters2,
                                std::vector<sensor_msgs::PointCloud> &clusters)
    {
      clusters.resize (clusters2.size ());
      for (size_t i = 0; i < clusters2.size (); ++i)
      {
        pcl16::PointCloud<PointT> cloud_cluster;
        pcl16::copyPointCloud (cloud_objects, clusters2[i], cloud_cluster);
        sensor_msgs::PointCloud2 pc2;
        pcl16::toROSMsg (cloud_cluster, pc2);
        sensor_msgs::convertPointCloud2ToPointCloud (pc2, clusters[i]);
      }
    }

  void
  TabletopSegmentor::processCloud (const sensor_msgs::PointCloud2 &cloud, TabletopSegmentation::Response &response,
                                   Table input_table)
  {
    ROS_INFO( "Starting process on new cloud in frame %s", cloud.header.frame_id.c_str());

    // pcl16 objects
    KdTreePtr normals_tree_, clusters_tree_;
    pcl16::VoxelGrid<Point> grid_, grid_objects_;
    //++Kadir
//      pcl16::PassThrough<Point> pass_;
    pcl16::PassThrough<Point> pass_ (true);
    //--Kadir
    pcl16::NormalEstimation<Point, pcl16::Normal> n3d_;
    pcl16::SACSegmentationFromNormals<Point, pcl16::Normal> seg_;
    pcl16::ProjectInliers<Point> proj_;
    pcl16::ConvexHull<Point> hull_;
    pcl16::ExtractPolygonalPrismData<Point> prism_;
    pcl16::EuclideanClusterExtraction<Point> pcl16_cluster_;
    pcl16::PointCloud<Point>::Ptr table_hull_ptr (new pcl16::PointCloud<Point>);

    // Filtering parameters
    grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
    grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
    grid_.setFilterFieldName ("z");
    grid_.setFilterLimits (z_filter_min_, z_filter_max_);
    grid_.setDownsampleAllData (false);
    grid_objects_.setDownsampleAllData (true);

    normals_tree_ = boost::make_shared<pcl16::search::KdTree<Point> > ();
    clusters_tree_ = boost::make_shared<pcl16::search::KdTree<Point> > ();

    // Normal estimation parameters
    n3d_.setKSearch (10);
    n3d_.setSearchMethod (normals_tree_);
    // Table model fitting parameters
    seg_.setDistanceThreshold (0.05);
    seg_.setMaxIterations (10000);
    seg_.setNormalDistanceWeight (0.1);
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl16::SACMODEL_NORMAL_PLANE);
    seg_.setMethodType (pcl16::SAC_RANSAC);
    seg_.setProbability (0.99);

    proj_.setModelType (pcl16::SACMODEL_PLANE);

    // Clustering parameters
    pcl16_cluster_.setClusterTolerance (cluster_distance_);
    pcl16_cluster_.setMinClusterSize (min_cluster_size_);
    pcl16_cluster_.setSearchMethod (clusters_tree_);

    // Step 1 : Filter, remove NaNs and downsample
    pcl16::PointCloud<Point>::Ptr cloud_ptr (new pcl16::PointCloud<Point>);
    pcl16::fromROSMsg (cloud, *cloud_ptr);
    pass_.setInputCloud (cloud_ptr);
    pass_.setFilterFieldName ("z");
    pass_.setFilterLimits (z_filter_min_, z_filter_max_);
    pcl16::PointCloud<Point>::Ptr z_cloud_filtered_ptr (new pcl16::PointCloud<Point>);
    pass_.filter (*z_cloud_filtered_ptr);

    //++Kadir
    pcl16::IndicesConstPtr removed_indices_ptr = pass_.getRemovedIndices ();
//    std::vector<int> removed_indices;
    std::vector<int> remaining_indices;
//    removed_indices = *removed_indices_ptr;
    //remove removed indices
    std::vector<int> tmp_indices;

    pcl16::IndicesPtr remaining_indices_ptr = boost::make_shared<std::vector<int> > ();
    remaining_indices_ptr->resize (cloud_ptr->points.size () - removed_indices_ptr->size ());
    uint start = 0, end = 0, cnt = 0;
    for (uint i = 0; i < removed_indices_ptr->size (); i++)
    {
      end = (*removed_indices_ptr)[i];
      for (uint j = start; j < end; j++)
      {
        (*remaining_indices_ptr)[cnt] = j;
        cnt++;
      }
      start = end + 1;
    }
    end = cloud_ptr->points.size ();
    for (uint j = start; j < end; j++)
    {
      (*remaining_indices_ptr)[cnt] = j;
      cnt++;
    }

    std::cout << "before_z : " << cloud_ptr->points.size () << "\n";
    std::cout << "removed_z: " << removed_indices_ptr->size () << "\n";
    std::cout << "after_z  : " << remaining_indices_ptr->size () << "\n";

//    pass_.setIndices (remaining_indices_ptr);
    pass_.setInputCloud (z_cloud_filtered_ptr);
    pass_.setFilterFieldName ("y");
    pass_.setFilterLimits (y_filter_min_, y_filter_max_);
    pcl16::PointCloud<Point>::Ptr y_cloud_filtered_ptr (new pcl16::PointCloud<Point>);
    pass_.filter (*y_cloud_filtered_ptr);
    removed_indices_ptr = pass_.getRemovedIndices ();
//    removed_indices = *removed_indices_ptr;
    remaining_indices = *remaining_indices_ptr;

    start = 0;
    for (uint i = 0; i < removed_indices_ptr->size (); i++)
    {
      end = (*removed_indices_ptr)[i];
      if (end > start)
        tmp_indices.insert (tmp_indices.end (), remaining_indices.begin () + start, remaining_indices.begin () + end);
      start = end + 1;
    }
    end = remaining_indices.size ();
    if (end > start)
      tmp_indices.insert (tmp_indices.end (), remaining_indices.begin () + start, remaining_indices.begin () + end);

    remaining_indices = tmp_indices;
    std::cout << "before_y : " << z_cloud_filtered_ptr->points.size () << "\n";
    std::cout << "removed_y: " << removed_indices_ptr->size () << "\n";
    std::cout << "after_y  : " << remaining_indices.size () << "\n";

    pass_.setInputCloud (y_cloud_filtered_ptr);
    pass_.setFilterFieldName ("x");
    pass_.setFilterLimits (x_filter_min_, x_filter_max_);
    pcl16::PointCloud<Point>::Ptr cloud_filtered_ptr (new pcl16::PointCloud<Point>);
    pass_.filter (*cloud_filtered_ptr);
    removed_indices_ptr = pass_.getRemovedIndices ();
    tmp_indices.clear ();

    start = 0;
    for (uint i = 0; i < removed_indices_ptr->size (); i++)
    {
      end = (*removed_indices_ptr)[i];
      if (end > start)
        tmp_indices.insert (tmp_indices.end (), remaining_indices.begin () + start, remaining_indices.begin () + end);
      start = end + 1;
    }
    end = remaining_indices.size ();
    if (end > start)
      tmp_indices.insert (tmp_indices.end (), remaining_indices.begin () + start, remaining_indices.begin () + end);
    remaining_indices = tmp_indices;
    std::cout << "before_x : " << y_cloud_filtered_ptr->points.size () << "\n";
    std::cout << "removed_x: " << removed_indices_ptr->size () << "\n";
    std::cout << "after_x  : " << remaining_indices.size () << "\n";
    std::cout << "cloud_filtered_ptr: " << cloud_filtered_ptr->points.size () << "\n";
    //--Kadir

    ROS_INFO("Step 1 done");
    if (cloud_filtered_ptr->points.size () < (unsigned int)min_cluster_size_)
    {
      ROS_INFO( "Filtered cloud only has %d points", (int)cloud_filtered_ptr->points.size());
      response.result = response.NO_TABLE;
      return;
    }

    pcl16::PointCloud<Point>::Ptr cloud_downsampled_ptr (new pcl16::PointCloud<Point>);
    grid_.setInputCloud (cloud_filtered_ptr);
    grid_.filter (*cloud_downsampled_ptr);
    if (cloud_downsampled_ptr->points.size () < (unsigned int)min_cluster_size_)
    {
      ROS_INFO( "Downsampled cloud only has %d points", (int)cloud_downsampled_ptr->points.size());
      response.result = response.NO_TABLE;
      return;
    }

    // Step 2 : Estimate normals
    pcl16::PointCloud<pcl16::Normal>::Ptr cloud_normals_ptr (new pcl16::PointCloud<pcl16::Normal>);
    n3d_.setInputCloud (cloud_downsampled_ptr);
    n3d_.compute (*cloud_normals_ptr);
    ROS_INFO("Step 2 done");

    // Step 3 : Perform planar segmentation, if table is not given, otherwise use given table
    tf::Transform table_plane_trans;
    tf::Transform table_plane_trans_flat;
    if (input_table.convex_hull.vertices.size () != 0)
    {
      ROS_INFO("Table input, skipping Step 3");
      bool success = tableMsgToPointCloud<pcl16::PointCloud<Point> > (input_table, cloud.header.frame_id,
                                                                      *table_hull_ptr);
      if (!success)
      {
        ROS_ERROR("Failure in converting table convex hull!");
        return;
      }
      response.table = input_table;
      if (flatten_table_)
      {
        ROS_ERROR("flatten_table mode is disabled if table is given!");
        flatten_table_ = false;
      }
    }
    else
    {
      pcl16::PointIndices::Ptr table_inliers_ptr (new pcl16::PointIndices);
      pcl16::ModelCoefficients::Ptr table_coefficients_ptr (new pcl16::ModelCoefficients);
      seg_.setInputCloud (cloud_downsampled_ptr);
      seg_.setInputNormals (cloud_normals_ptr);
      seg_.segment (*table_inliers_ptr, *table_coefficients_ptr);

      if (table_coefficients_ptr->values.size () <= 3)
      {
        ROS_INFO("Failed to detect table in scan");
        response.result = response.NO_TABLE;
        return;
      }

      if (table_inliers_ptr->indices.size () < (unsigned int)inlier_threshold_)
      {
        ROS_INFO(
            "Plane detection has %d inliers, below min threshold of %d", (int)table_inliers_ptr->indices.size(), inlier_threshold_);
        response.result = response.NO_TABLE;
        return;
      }

      ROS_INFO(
          "[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].", (int)table_inliers_ptr->indices.size (), table_coefficients_ptr->values[0], table_coefficients_ptr->values[1], table_coefficients_ptr->values[2], table_coefficients_ptr->values[3]);
      ROS_INFO("Step 3 done");

      // Step 4 : Project the table inliers on the table
      pcl16::PointCloud<Point>::Ptr table_projected_ptr (new pcl16::PointCloud<Point>);
      proj_.setInputCloud (cloud_downsampled_ptr);
      proj_.setIndices (table_inliers_ptr);
      proj_.setModelCoefficients (table_coefficients_ptr);
      proj_.filter (*table_projected_ptr);
      ROS_INFO("Step 4 done");

      sensor_msgs::PointCloud table_points;
      sensor_msgs::PointCloud table_hull_points;
      table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_, false);

      // ---[ Estimate the convex hull (not in table frame)
      hull_.setInputCloud (table_projected_ptr);
      hull_.reconstruct (*table_hull_ptr);

      if (!flatten_table_)
      {
        // --- [ Take the points projected on the table and transform them into the PointCloud message
        //  while also transforming them into the table's coordinate system
        if (!getPlanePoints<Point> (*table_projected_ptr, table_plane_trans, table_points))
        {
          response.result = response.OTHER_ERROR;
          return;
        }

        // ---[ Create the table message
        response.table = getTable<sensor_msgs::PointCloud> (cloud.header, table_plane_trans, table_points);

        // ---[ Convert the convex hull points to table frame
        if (!getPlanePoints<Point> (*table_hull_ptr, table_plane_trans, table_hull_points))
        {
          response.result = response.OTHER_ERROR;
          return;
        }
      }
      if (flatten_table_)
      {
        // if flattening the table, find the center of the convex hull and move the table frame there
        table_plane_trans_flat = getPlaneTransform (*table_coefficients_ptr, up_direction_, flatten_table_);
        tf::Vector3 flat_table_pos;
        double avg_x, avg_y, avg_z;
        avg_x = avg_y = avg_z = 0;
        for (size_t i = 0; i < table_projected_ptr->points.size (); i++)
        {
          avg_x += table_projected_ptr->points[i].x;
          avg_y += table_projected_ptr->points[i].y;
          avg_z += table_projected_ptr->points[i].z;
        }
        avg_x /= table_projected_ptr->points.size ();
        avg_y /= table_projected_ptr->points.size ();
        avg_z /= table_projected_ptr->points.size ();
        ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x, avg_y, avg_z);

        // place the new table frame in the center of the convex hull
        flat_table_pos[0] = avg_x;
        flat_table_pos[1] = avg_y;
        flat_table_pos[2] = avg_z;
        table_plane_trans_flat.setOrigin (flat_table_pos);

        // shift the non-flat table frame to the center of the convex hull as well
        table_plane_trans.setOrigin (flat_table_pos);

        // --- [ Take the points projected on the table and transform them into the PointCloud message
        //  while also transforming them into the flat table's coordinate system
        sensor_msgs::PointCloud flat_table_points;
        if (!getPlanePoints<Point> (*table_projected_ptr, table_plane_trans_flat, flat_table_points))
        {
          response.result = response.OTHER_ERROR;
          return;
        }

        // ---[ Create the table message
        response.table = getTable<sensor_msgs::PointCloud> (cloud.header, table_plane_trans_flat, flat_table_points);

        // ---[ Convert the convex hull points to flat table frame
        if (!getPlanePoints<Point> (*table_hull_ptr, table_plane_trans_flat, table_hull_points))
        {
          response.result = response.OTHER_ERROR;
          return;
        }
      }

      ROS_INFO("Table computed");
      // ---[ Add the convex hull as a triangle mesh to the Table message
      addConvexHullTable<sensor_msgs::PointCloud> (response.table, table_hull_points, flatten_table_);
    }

    // Step 5: Get the objects on top of the (non-flat) table
    pcl16::PointIndices cloud_object_indices;
    //prism_.setInputCloud (cloud_all_minus_table_ptr);
    prism_.setInputCloud (cloud_filtered_ptr);
    prism_.setInputPlanarHull (table_hull_ptr);
    ROS_INFO( "Using table prism: %f to %f", table_z_filter_min_, table_z_filter_max_);
    prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);
    prism_.segment (cloud_object_indices);
    //++Kadir
    tmp_indices.resize (cloud_object_indices.indices.size ());
    for (uint i = 0; i < tmp_indices.size (); i++)
      tmp_indices[i] = remaining_indices[cloud_object_indices.indices[i]];
    remaining_indices = tmp_indices;
    //--Kadir

    pcl16::PointCloud<Point>::Ptr cloud_objects_ptr (new pcl16::PointCloud<Point>);
    pcl16::ExtractIndices<Point> extract_object_indices;
    extract_object_indices.setInputCloud (cloud_filtered_ptr);
    extract_object_indices.setIndices (boost::make_shared<const pcl16::PointIndices> (cloud_object_indices));
    extract_object_indices.filter (*cloud_objects_ptr);

    ROS_INFO( " Number of object point candidates: %d.", (int)cloud_objects_ptr->points.size ());
    response.result = response.SUCCESS;

    //++Kadir
//    pcl16::PointCloud<Point>::Ptr debug_ptr (new pcl16::PointCloud<Point>);
//    extract_object_indices.setInputCloud (cloud_ptr);
//    pcl16::PointIndices debug_indices;
//    debug_indices.indices = tmp_indices;
//    extract_object_indices.setIndices (boost::make_shared<const pcl16::PointIndices> (debug_indices));
//    extract_object_indices.filter (*debug_ptr);
//    sensor_msgs::PointCloud2 msg;
//    pcl16::toROSMsg (*debug_ptr, msg);
//    .publish (msg);
    //--Kadir

    if (cloud_objects_ptr->points.empty ())
    {
      ROS_INFO("No objects on table");
      return;
    }

    // ---[ Downsample the points
    pcl16::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl16::PointCloud<Point>);
    grid_objects_.setInputCloud (cloud_objects_ptr);
    grid_objects_.filter (*cloud_objects_downsampled_ptr);
    std::cout << "n_grids: " << grid_objects_.map_gid_2_pc_indices_.size () << std::endl;
    std::map<int, std::vector<int> >::iterator m_it;
    int n_indices = 0;
    for (m_it = grid_objects_.map_gid_2_pc_indices_.begin (); m_it != grid_objects_.map_gid_2_pc_indices_.end ();
        m_it++)
    {
//      std::cout << "grid_id: " << m_it->first << "\n";
      n_indices += m_it->second.size ();
//      for (uint i = 0; i < m_it->second.size (); i++)
//      {
//        std::cout << "\tindices: " << m_it->second[i] << " " << remaining_indices[m_it->second[i]] << "\n";
//      }
//      std::cout << "\n";
    }
    std::cout << "n_indices: " << n_indices << "\n";
    std::cout << "n_grids  : " << grid_objects_.map_gid_2_pc_indices_.size () << std::endl;

    // ---[ If flattening the table, adjust the points on the table to be straight also
    if (flatten_table_)
      straightenPoints<pcl16::PointCloud<Point> > (*cloud_objects_downsampled_ptr, table_plane_trans,
                                                   table_plane_trans_flat);

    // Step 6: Split the objects into Euclidean clusters
    std::vector<pcl16::PointIndices> clusters2;
    //pcl16_cluster_.setInputCloud (cloud_objects_ptr);
    pcl16_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
    pcl16_cluster_.extract (clusters2);
    ROS_INFO( "Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

    // ---[ Convert clusters into the PointCloud message
    std::vector<sensor_msgs::PointCloud> clusters;
    getClustersFromPointCloud2<Point> (*cloud_objects_downsampled_ptr, clusters2, clusters);
    ROS_INFO("Clusters converted");
    response.clusters = clusters;
    response.header = cloud.header;

    // ++Kadir
    //clusters of grids
    response.clusters_indices.resize (clusters2.size ());
    for (uint i = 0; i < clusters2.size (); i++)
    {
      for (uint j = 0; j < clusters2[i].indices.size (); j++)
      {
        std::vector<int> tmp = grid_objects_.map_gid_2_pc_indices_[clusters2[i].indices[j]];
        std::vector<int> tmp_indices (tmp.size ());
        for (uint k = 0; k < tmp.size (); k++)
          tmp_indices[k] = remaining_indices[tmp[k]];
        response.clusters_indices[i].indices.insert (response.clusters_indices[i].indices.end (), tmp_indices.begin (),
                                                     tmp_indices.end ());
      }
    }
    extract_object_indices.setInputCloud (cloud_ptr);
    for (uint i = 0; i < response.clusters_indices.size (); i++)
      std::cout << "cluster_indices_" << i << "\t" << response.clusters_indices[i].indices.size () << std::endl;
    for (uint i = 0; i < response.clusters_indices.size () && i < pubs_clusters_.size (); i++)
    {
      pcl16::PointCloud<Point>::Ptr debug_ptr (new pcl16::PointCloud<Point>);
      pcl16::PointIndices debug_indices;
      debug_indices.indices = response.clusters_indices[i].indices;
      extract_object_indices.setIndices (boost::make_shared<const pcl16::PointIndices> (debug_indices));
      extract_object_indices.filter (*debug_ptr);
      sensor_msgs::PointCloud2 msg;
      pcl16::toROSMsg (*debug_ptr, msg);
      pubs_clusters_[i].publish (msg);
    }
    response.cloud = cloud;
    // --Kadir

    publishClusterMarkers (clusters, cloud.header);
  }

} //namespace tabletop_object_segmentation

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "tabletop_segmentation_node");
  ros::NodeHandle nh;

  tabletop_object_segmentation::TabletopSegmentor node (nh);
  ros::spin ();
  return 0;
}
