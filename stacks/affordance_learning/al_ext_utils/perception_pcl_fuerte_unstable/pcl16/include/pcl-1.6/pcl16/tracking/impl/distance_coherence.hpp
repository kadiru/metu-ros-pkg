#ifndef PCL16_TRACKING_IMPL_DISTANCE_COHERENCE_H_
#define PCL16_TRACKING_IMPL_DISTANCE_COHERENCE_H_

#include <Eigen/Dense>

namespace pcl16
{
  namespace tracking
  {
    template <typename PointInT> double
    DistanceCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
    {
       Eigen::Vector4f p = source.getVector4fMap ();
       Eigen::Vector4f p_dash = target.getVector4fMap ();
       double d = (p - p_dash).norm ();
       return 1.0 / (1.0 + d * d * weight_);
    }
  }
}

#define PCL16_INSTANTIATE_DistanceCoherence(T) template class PCL16_EXPORTS pcl16::tracking::DistanceCoherence<T>;

#endif
