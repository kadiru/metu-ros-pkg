#include <iostream>
#include <pcl16/point_types.h>
#include <pcl16/filters/radius_outlier_removal.h>

int
 main (int argc, char** argv)
{
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>);
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered (new pcl16::PointCloud<pcl16::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  // build the filter
  pcl16::RadiusOutlierRemoval<pcl16::PointXYZ> outrem;
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (2);

  // apply filter
  outrem.filter (*cloud_filtered);

  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}
