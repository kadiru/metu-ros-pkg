#ifndef PCL16_TRACKING_IMPL_TRACKER_H_
#define PCL16_TRACKING_IMPL_TRACKER_H_

#include <boost/random.hpp>
#include <pcl16/common/eigen.h>
#include <ctime>

template <typename PointInT, typename StateT> bool
pcl16::tracking::Tracker<PointInT, StateT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL16_ERROR ("[pcl16::%s::initCompute] PCLBase::Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL16_ERROR ("[pcl16::%s::compute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }

  return (true);
}


template <typename PointInT, typename StateT> void
pcl16::tracking::Tracker<PointInT, StateT>::compute ()
{
  if (!initCompute ())
    return;
  
  computeTracking ();
  deinitCompute ();
}

#endif
