#include <pcl16/ModelCoefficients.h>
#include <pcl16/point_types.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/filters/voxel_grid.h>
#include <pcl16/features/normal_3d.h>
#include <pcl16/kdtree/kdtree.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/segmentation/extract_clusters.h>


int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl16::PCDReader reader;
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>), cloud_f (new pcl16::PointCloud<pcl16::PointXYZ>);
  reader.read ("table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl16::VoxelGrid<pcl16::PointXYZ> vg;
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered (new pcl16::PointCloud<pcl16::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl16::SACSegmentation<pcl16::PointXYZ> seg;
  pcl16::PointIndices::Ptr inliers (new pcl16::PointIndices);
  pcl16::ModelCoefficients::Ptr coefficients (new pcl16::ModelCoefficients);
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_plane (new pcl16::PointCloud<pcl16::PointXYZ> ());
  pcl16::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl16::SACMODEL_PLANE);
  seg.setMethodType (pcl16::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl16::ExtractIndices<pcl16::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered = cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl16::search::KdTree<pcl16::PointXYZ>::Ptr tree (new pcl16::search::KdTree<pcl16::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl16::PointIndices> cluster_indices;
  pcl16::EuclideanClusterExtraction<pcl16::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl16::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_cluster (new pcl16::PointCloud<pcl16::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl16::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
