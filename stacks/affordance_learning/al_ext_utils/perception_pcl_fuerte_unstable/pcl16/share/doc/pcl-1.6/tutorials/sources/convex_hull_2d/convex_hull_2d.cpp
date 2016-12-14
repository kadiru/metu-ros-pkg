#include <pcl16/ModelCoefficients.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/point_types.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/filters/passthrough.h>
#include <pcl16/filters/project_inliers.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/surface/convex_hull.h>

int
 main (int argc, char** argv)
{
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>), cloud_filtered (new pcl16::PointCloud<pcl16::PointXYZ>), cloud_projected (new pcl16::PointCloud<pcl16::PointXYZ>);
  pcl16::PCDReader reader;
  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);

  // Build a filter to remove spurious NaNs
  pcl16::PassThrough<pcl16::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  pcl16::ModelCoefficients::Ptr coefficients (new pcl16::ModelCoefficients);
  pcl16::PointIndices::Ptr inliers (new pcl16::PointIndices);
  // Create the segmentation object
  pcl16::SACSegmentation<pcl16::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl16::SACMODEL_PLANE);
  seg.setMethodType (pcl16::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Project the model inliers
  pcl16::ProjectInliers<pcl16::PointXYZ> proj;
  proj.setModelType (pcl16::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Convex Hull representation of the projected inliers
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_hull (new pcl16::PointCloud<pcl16::PointXYZ>);
  pcl16::ConvexHull<pcl16::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

  pcl16::PCDWriter writer;
  writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

  return (0);
}
