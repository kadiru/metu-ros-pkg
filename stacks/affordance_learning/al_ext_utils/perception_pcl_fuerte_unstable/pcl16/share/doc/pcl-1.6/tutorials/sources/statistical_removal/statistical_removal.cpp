#include <iostream>
#include <pcl16/io/pcd_io.h>
#include <pcl16/point_types.h>
#include <pcl16/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>);
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered (new pcl16::PointCloud<pcl16::PointXYZ>);

  // Fill in the cloud data
  pcl16::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl16::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl16::StatisticalOutlierRemoval<pcl16::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl16::PCDWriter writer;
  writer.write<pcl16::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl16::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);
}