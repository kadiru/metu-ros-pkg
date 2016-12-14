/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl16/common/common_headers.h>
#include <pcl16/common/common_headers.h>
#include <pcl16/features/normal_3d.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/visualization/pcl_visualizer.h>
#include <pcl16/console/parse.h>

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}


boost::shared_ptr<pcl16::visualization::PCLVisualizer> simpleVis (pcl16::PointCloud<pcl16::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer (new pcl16::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl16::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl16::visualization::PCL16_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl16::visualization::PCLVisualizer> rgbVis (pcl16::PointCloud<pcl16::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer (new pcl16::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl16::visualization::PointCloudColorHandlerRGBField<pcl16::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl16::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl16::visualization::PCL16_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl16::visualization::PCLVisualizer> customColourVis (pcl16::PointCloud<pcl16::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer (new pcl16::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl16::visualization::PointCloudColorHandlerCustom<pcl16::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl16::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl16::visualization::PCL16_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl16::visualization::PCLVisualizer> normalsVis (
    pcl16::PointCloud<pcl16::PointXYZRGB>::ConstPtr cloud, pcl16::PointCloud<pcl16::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer (new pcl16::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl16::visualization::PointCloudColorHandlerRGBField<pcl16::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl16::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl16::visualization::PCL16_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl16::PointXYZRGB, pcl16::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl16::visualization::PCLVisualizer> shapesVis (pcl16::PointCloud<pcl16::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer (new pcl16::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl16::visualization::PointCloudColorHandlerRGBField<pcl16::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl16::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl16::visualization::PCL16_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl16::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl16::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}


boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewportsVis (
    pcl16::PointCloud<pcl16::PointXYZRGB>::ConstPtr cloud, pcl16::PointCloud<pcl16::Normal>::ConstPtr normals1, pcl16::PointCloud<pcl16::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer (new pcl16::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl16::visualization::PointCloudColorHandlerRGBField<pcl16::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl16::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl16::visualization::PointCloudColorHandlerCustom<pcl16::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl16::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl16::visualization::PCL16_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl16::visualization::PCL16_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl16::PointXYZRGB, pcl16::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl16::PointXYZRGB, pcl16::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl16::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl16::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl16::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl16::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl16::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl16::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl16::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer (new pcl16::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl16::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool simple(false), rgb(false), custom_c(false), normals(false),
    shapes(false), viewports(false), interaction_customization(false);
  if (pcl16::console::find_argument (argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation example\n";
  }
  else if (pcl16::console::find_argument (argc, argv, "-c") >= 0)
  {
    custom_c = true;
    std::cout << "Custom colour visualisation example\n";
  }
  else if (pcl16::console::find_argument (argc, argv, "-r") >= 0)
  {
    rgb = true;
    std::cout << "RGB colour visualisation example\n";
  }
  else if (pcl16::console::find_argument (argc, argv, "-n") >= 0)
  {
    normals = true;
    std::cout << "Normals visualisation example\n";
  }
  else if (pcl16::console::find_argument (argc, argv, "-a") >= 0)
  {
    shapes = true;
    std::cout << "Shapes visualisation example\n";
  }
  else if (pcl16::console::find_argument (argc, argv, "-v") >= 0)
  {
    viewports = true;
    std::cout << "Viewports example\n";
  }
  else if (pcl16::console::find_argument (argc, argv, "-i") >= 0)
  {
    interaction_customization = true;
    std::cout << "Interaction Customization example\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl16::PointCloud<pcl16::PointXYZ>::Ptr basic_cloud_ptr (new pcl16::PointCloud<pcl16::PointXYZ>);
  pcl16::PointCloud<pcl16::PointXYZRGB>::Ptr point_cloud_ptr (new pcl16::PointCloud<pcl16::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl16::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl16::deg2rad(angle));
      basic_point.y = sinf (pcl16::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl16::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl16::NormalEstimation<pcl16::PointXYZRGB, pcl16::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl16::search::KdTree<pcl16::PointXYZRGB>::Ptr tree (new pcl16::search::KdTree<pcl16::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl16::PointCloud<pcl16::Normal>::Ptr cloud_normals1 (new pcl16::PointCloud<pcl16::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl16::PointCloud<pcl16::Normal>::Ptr cloud_normals2 (new pcl16::PointCloud<pcl16::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);

  boost::shared_ptr<pcl16::visualization::PCLVisualizer> viewer;
  if (simple)
  {
    viewer = simpleVis(basic_cloud_ptr);
  }
  else if (rgb)
  {
    viewer = rgbVis(point_cloud_ptr);
  }
  else if (custom_c)
  {
    viewer = customColourVis(basic_cloud_ptr);
  }
  else if (normals)
  {
    viewer = normalsVis(point_cloud_ptr, cloud_normals2);
  }
  else if (shapes)
  {
    viewer = shapesVis(point_cloud_ptr);
  }
  else if (viewports)
  {
    viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
  }
  else if (interaction_customization)
  {
    viewer = interactionCustomizationVis();
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
