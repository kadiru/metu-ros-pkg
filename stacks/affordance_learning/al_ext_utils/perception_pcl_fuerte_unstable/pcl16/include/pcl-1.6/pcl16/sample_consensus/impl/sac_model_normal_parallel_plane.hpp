/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009-2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: sac_model_normal_parallel_plane.hpp 5026 2012-03-12 02:51:44Z rusu $
 *
 */

#ifndef PCL16_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_
#define PCL16_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_

#include <pcl16/sample_consensus/sac_model_normal_parallel_plane.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl16::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
{
  if (!normals_)
  {
    PCL16_ERROR ("[pcl16::SampleConsensusModelNormalParallelPlane::selectWithinDistance] No input dataset containing normals was given!\n");
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  // Obtain the plane normal
  Eigen::Vector4f coeff = model_coefficients;

  int nr_p = 0;
  inliers.resize (indices_->size ());
  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 1);
    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (coeff.dot (p));

    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = getAngle3D (n, coeff);
    d_normal = (std::min) (d_normal, fabs(M_PI - d_normal));

    if (fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid) < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      nr_p++;
    }
  }
  inliers.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> int
pcl16::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold)
{
  if (!normals_)
  {
    PCL16_ERROR ("[pcl16::SampleConsensusModelNormalParallelPlane::countWithinDistance] No input dataset containing normals was given!\n");
    return (0);
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  // Obtain the plane normal
  Eigen::Vector4f coeff = model_coefficients;

  int nr_p = 0;

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 1);
    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (coeff.dot (p));

    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = fabs (getAngle3D (n, coeff));
    d_normal = (std::min) (d_normal, fabs(M_PI - d_normal));

    if (fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid) < threshold)
      nr_p++;
  }
  return (nr_p);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl16::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  if (!normals_)
  {
    PCL16_ERROR ("[pcl16::SampleConsensusModelNormalParallelPlane::getDistancesToModel] No input dataset containing normals was given!\n");
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  // Obtain the plane normal
  Eigen::Vector4f coeff = model_coefficients;

  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 1);
    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (coeff.dot (p));

    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = getAngle3D (n, coeff);
    d_normal = (std::min) (d_normal, fabs (M_PI - d_normal));

    distances[i] = fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl16::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    PCL16_ERROR ("[pcl16::SampleConsensusModelNormalParallelPlane::isModelValid] Invalid number of model coefficients given (%zu)!\n", model_coefficients.size ());
    return (false);
  }

  // Check against template, if given
  if (eps_angle_ > 0.0)
  {
    // Obtain the plane normal
    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;
    coeff.normalize ();

    if (fabs (axis_.dot (coeff)) < cos_angle_)
      return  (false);
  }

  if (eps_dist_ > 0.0)
  {
    if (fabs (-model_coefficients[3] - distance_from_origin_) > eps_dist_)
      return (false);
  }

  return (true);
}

#define PCL16_INSTANTIATE_SampleConsensusModelNormalParallelPlane(PointT, PointNT) template class PCL16_EXPORTS pcl16::SampleConsensusModelNormalParallelPlane<PointT, PointNT>;

#endif    // PCL16_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_

