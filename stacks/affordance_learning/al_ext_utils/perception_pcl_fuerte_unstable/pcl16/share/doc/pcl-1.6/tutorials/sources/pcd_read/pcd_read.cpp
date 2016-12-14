#include <iostream>
#include <pcl16/io/pcd_io.h>
#include <pcl16/point_types.h>

int
main (int argc, char** argv)
{
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>);

  if (pcl16::io::loadPCDFile<pcl16::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL16_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;

  return (0);
}
