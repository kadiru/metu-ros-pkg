#include <pcl16/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl16/io/io.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/features/integral_image_normal.h>
    
int 
main ()
{
    // load point cloud
    pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZ>);
    pcl16::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
    
    // estimate normals
    pcl16::PointCloud<pcl16::Normal>::Ptr normals (new pcl16::PointCloud<pcl16::Normal>);

    pcl16::IntegralImageNormalEstimation<pcl16::PointXYZ, pcl16::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl16::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl16::PointXYZ,pcl16::Normal>(cloud, normals);
    
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
    return 0;
}
