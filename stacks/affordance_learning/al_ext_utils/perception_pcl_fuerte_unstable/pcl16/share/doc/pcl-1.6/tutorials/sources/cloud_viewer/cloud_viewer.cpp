#include <pcl16/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl16/io/io.h>
#include <pcl16/io/pcd_io.h>
    
int user_data;
    
void 
viewerOneOff (pcl16::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl16::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void 
viewerPsycho (pcl16::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    
int 
main ()
{
    pcl16::PointCloud<pcl16::PointXYZRGBA>::Ptr cloud (new pcl16::PointCloud<pcl16::PointXYZRGBA>);
    pcl16::io::loadPCDFile ("my_point_cloud.pcd", *cloud);
    
    pcl16::visualization::CloudViewer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
    return 0;
}
