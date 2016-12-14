#include <iostream>
#include <pcl16/io/pcd_io.h>
#include <pcl16/point_types.h>
#include <pcl16/ModelCoefficients.h>
#include <pcl16/filters/project_inliers.h>

int
 main (int argc, char** argv)
{
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>);
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_projected (new pcl16::PointCloud<pcl16::PointXYZ>);

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

  std::cerr << "Cloud before projection: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl16::ModelCoefficients::Ptr coefficients (new pcl16::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Create the filtering object
  pcl16::ProjectInliers<pcl16::PointXYZ> proj;
  proj.setModelType (pcl16::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  std::cerr << "Cloud after projection: " << std::endl;
  for (size_t i = 0; i < cloud_projected->points.size (); ++i)
    std::cerr << "    " << cloud_projected->points[i].x << " " 
                        << cloud_projected->points[i].y << " " 
                        << cloud_projected->points[i].z << std::endl;

  return (0);
}
