#include <iostream>
#include <pcl16/ModelCoefficients.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/point_types.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/filters/voxel_grid.h>
#include <pcl16/filters/extract_indices.h>

int
main (int argc, char** argv)
{
  sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered (new pcl16::PointCloud<pcl16::PointXYZ>), cloud_p (new pcl16::PointCloud<pcl16::PointXYZ>), cloud_f (new pcl16::PointCloud<pcl16::PointXYZ>);

  // Fill in the cloud data
  pcl16::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl16::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl16::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl16::PCDWriter writer;
  writer.write<pcl16::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl16::ModelCoefficients::Ptr coefficients (new pcl16::ModelCoefficients ());
  pcl16::PointIndices::Ptr inliers (new pcl16::PointIndices ());
  // Create the segmentation object
  pcl16::SACSegmentation<pcl16::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl16::SACMODEL_PLANE);
  seg.setMethodType (pcl16::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl16::ExtractIndices<pcl16::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl16::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

  return (0);
}
