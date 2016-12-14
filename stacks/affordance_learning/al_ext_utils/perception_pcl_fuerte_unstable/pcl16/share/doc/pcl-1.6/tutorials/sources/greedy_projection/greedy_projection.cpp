#include <pcl16/point_types.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/kdtree/kdtree_flann.h>
#include <pcl16/features/normal_3d.h>
#include <pcl16/surface/gp3.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>);
  sensor_msgs::PointCloud2 cloud_blob;
  pcl16::io::loadPCDFile ("bun0.pcd", cloud_blob);
  pcl16::fromROSMsg (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  pcl16::NormalEstimation<pcl16::PointXYZ, pcl16::Normal> n;
  pcl16::PointCloud<pcl16::Normal>::Ptr normals (new pcl16::PointCloud<pcl16::Normal>);
  pcl16::search::KdTree<pcl16::PointXYZ>::Ptr tree (new pcl16::search::KdTree<pcl16::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl16::PointCloud<pcl16::PointNormal>::Ptr cloud_with_normals (new pcl16::PointCloud<pcl16::PointNormal>);
  pcl16::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl16::search::KdTree<pcl16::PointNormal>::Ptr tree2 (new pcl16::search::KdTree<pcl16::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl16::GreedyProjectionTriangulation<pcl16::PointNormal> gp3;
  pcl16::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Finish
  return (0);
}
