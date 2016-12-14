#include <pcl16/point_types.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/kdtree/kdtree_flann.h>
#include <pcl16/surface/mls.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ> ());
  // Load bun0.pcd -- should be available with the PCL archive in test 
  pcl16::io::loadPCDFile ("bun0.pcd", *cloud);

  // Create a KD-Tree
  pcl16::search::KdTree<pcl16::PointXYZ>::Ptr tree (new pcl16::search::KdTree<pcl16::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl16::PointCloud<pcl16::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl16::MovingLeastSquares<pcl16::PointXYZ, pcl16::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl16::io::savePCDFile ("bun0-mls.pcd", mls_points);
}
