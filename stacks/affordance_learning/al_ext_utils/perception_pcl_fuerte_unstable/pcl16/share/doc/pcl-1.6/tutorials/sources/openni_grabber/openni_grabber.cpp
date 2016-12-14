#include <pcl16/point_cloud.h>
#include <pcl16/point_types.h>
#include <pcl16/io/openni_grabber.h>
#include <pcl16/common/time.h>

class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const pcl16::PointCloud<pcl16::PointXYZRGBA>::ConstPtr &cloud)
  {
    static unsigned count = 0;
    static double last = pcl16::getTime ();
    if (++count == 30)
    {
      double now = pcl16::getTime ();
      std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      count = 0;
      last = now;
    }
  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl16::Grabber* interface = new pcl16::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl16::PointCloud<pcl16::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (true)
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }
};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}
