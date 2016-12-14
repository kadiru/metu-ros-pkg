#include <pcl16/ModelCoefficients.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/point_types.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/filters/passthrough.h>
#include <pcl16/features/normal_3d.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/segmentation/sac_segmentation.h>

typedef pcl16::PointXYZ PointT;

int
main (int argc, char** argv)
{
  // All the objects needed
  pcl16::PCDReader reader;
  pcl16::PassThrough<PointT> pass;
  pcl16::NormalEstimation<PointT, pcl16::Normal> ne;
  pcl16::SACSegmentationFromNormals<PointT, pcl16::Normal> seg; 
  pcl16::PCDWriter writer;
  pcl16::ExtractIndices<PointT> extract;
  pcl16::ExtractIndices<pcl16::Normal> extract_normals;
  pcl16::search::KdTree<PointT>::Ptr tree (new pcl16::search::KdTree<PointT> ());

  // Datasets
  pcl16::PointCloud<PointT>::Ptr cloud (new pcl16::PointCloud<PointT>);
  pcl16::PointCloud<PointT>::Ptr cloud_filtered (new pcl16::PointCloud<PointT>);
  pcl16::PointCloud<pcl16::Normal>::Ptr cloud_normals (new pcl16::PointCloud<pcl16::Normal>);
  pcl16::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl16::PointCloud<PointT>);
  pcl16::PointCloud<pcl16::Normal>::Ptr cloud_normals2 (new pcl16::PointCloud<pcl16::Normal>);
  pcl16::ModelCoefficients::Ptr coefficients_plane (new pcl16::ModelCoefficients), coefficients_cylinder (new pcl16::ModelCoefficients);
  pcl16::PointIndices::Ptr inliers_plane (new pcl16::PointIndices), inliers_cylinder (new pcl16::PointIndices);

  // Read in the cloud data
  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl16::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl16::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl16::PointCloud<PointT>::Ptr cloud_plane (new pcl16::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl16::SACMODEL_CYLINDER);
  seg.setMethodType (pcl16::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl16::PointCloud<PointT>::Ptr cloud_cylinder (new pcl16::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	  writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  }
  return (0);
}
