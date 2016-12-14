/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: project_inliers.hpp 5026 2012-03-12 02:51:44Z rusu $
 *
 */

#ifndef PCL16_FILTERS_IMPL_PROJECT_INLIERS_H_
#define PCL16_FILTERS_IMPL_PROJECT_INLIERS_H_

#include <pcl16/filters/project_inliers.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl16::ProjectInliers<PointT>::applyFilter (PointCloud &output)
{
  if (indices_->empty ())
  {
    PCL16_WARN ("[pcl16::%s::applyFilter] No indices given or empty indices!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  //Eigen::Map<Eigen::VectorXf, Eigen::Aligned> model_coefficients (&model_->values[0], model_->values.size ());
  // More expensive than a map but safer (32bit architectures seem to complain)
  Eigen::VectorXf model_coefficients (model_->values.size ());
  for (size_t i = 0; i < model_->values.size (); ++i)
    model_coefficients[i] = model_->values[i];

  // Initialize the Sample Consensus model and set its parameters
  if (!initSACModel (model_type_))
  {
    PCL16_ERROR ("[pcl16::%s::segment] Error initializing the SAC model!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (copy_all_data_)
    sacmodel_->projectPoints (*indices_, model_coefficients, output, true);
  else
    sacmodel_->projectPoints (*indices_, model_coefficients, output, false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl16::ProjectInliers<PointT>::initSACModel (int model_type)
{
  // Build the model
  switch (model_type)
  {
    case SACMODEL_PLANE:
    {
      //PCL16_DEBUG ("[pcl16::%s::initSACModel] Using a model of type: SACMODEL_PLANE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelPlane<PointT> (input_));
      break;
    }
    case SACMODEL_LINE:
    {
      //PCL16_DEBUG ("[pcl16::%s::initSACModel] Using a model of type: SACMODEL_LINE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelLine<PointT> (input_));
      break;
    }
    case SACMODEL_CIRCLE2D:
    {
      //PCL16_DEBUG ("[pcl16::%s::initSACModel] Using a model of type: SACMODEL_CIRCLE2D\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelCircle2D<PointT> (input_));
      break;
    }
    case SACMODEL_SPHERE:
    {
      //PCL16_DEBUG ("[pcl16::%s::initSACModel] Using a model of type: SACMODEL_SPHERE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelSphere<PointT> (input_));
      break;
    }
    case SACMODEL_PARALLEL_LINE:
    {
      //PCL16_DEBUG ("[pcl16::%s::initSACModel] Using a model of type: SACMODEL_PARALLEL_LINE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelParallelLine<PointT> (input_));
      break;
    }
    case SACMODEL_PERPENDICULAR_PLANE:
    {
      //PCL16_DEBUG ("[pcl16::%s::initSACModel] Using a model of type: SACMODEL_PERPENDICULAR_PLANE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelPerpendicularPlane<PointT> (input_));
      break;
    }
    case SACMODEL_CYLINDER:
    {
      //PCL16_DEBUG ("[pcl16::%s::segment] Using a model of type: SACMODEL_CYLINDER\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelCylinder<PointT, pcl16::Normal> (input_));
      break;
    }
    case SACMODEL_NORMAL_PLANE:
    {
      //PCL16_DEBUG ("[pcl16::%s::segment] Using a model of type: SACMODEL_NORMAL_PLANE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelNormalPlane<PointT, pcl16::Normal> (input_));
      break;
    }
    case SACMODEL_CONE:
    {
      //PCL16_DEBUG ("[pcl16::%s::segment] Using a model of type: SACMODEL_CONE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelCone<PointT, pcl16::Normal> (input_));
      break;
    }
    case SACMODEL_NORMAL_SPHERE:
    {
      //PCL16_DEBUG ("[pcl16::%s::segment] Using a model of type: SACMODEL_NORMAL_SPHERE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelNormalSphere<PointT, pcl16::Normal> (input_));
      break;
    }
    case SACMODEL_NORMAL_PARALLEL_PLANE:
    {
      //PCL16_DEBUG ("[pcl16::%s::segment] Using a model of type: SACMODEL_NORMAL_PARALLEL_PLANE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelNormalParallelPlane<PointT, pcl16::Normal> (input_));
      break;
    }
    case SACMODEL_PARALLEL_PLANE:
    {
      //PCL16_DEBUG ("[pcl16::%s::segment] Using a model of type: SACMODEL_PARALLEL_PLANE\n", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelParallelPlane<PointT> (input_));
      break;
    }
    default:
    {
      PCL16_ERROR ("[pcl16::%s::initSACModel] No valid model given!\n", getClassName ().c_str ());
      return (false);
    }
  }
  return (true);
}

#define PCL16_INSTANTIATE_ProjectInliers(T) template class PCL16_EXPORTS pcl16::ProjectInliers<T>;

#endif    // PCL16_FILTERS_IMPL_PROJECT_INLIERS_H_

