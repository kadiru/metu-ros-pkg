/*
 * SurfaceFExtractor.h
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
 * kadir@ceng.metu.edu.tr
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Kovan Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SURFACEFEXTRACTOR_H_
#define SURFACEFEXTRACTOR_H_

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "al_perception2/feature_extractors/FeatureExtractor.h"
#include <pcl/features/principal_curvatures.h>
#include "pcl-1.6/pcl/features/principal_curvatures.h"
#include "pcl-1.6/pcl/features/impl/principal_curvatures.hpp"
#include <pcl/features/normal_3d_omp.h>
#include "ros/console.h"

namespace al
{
  namespace perception
  {
    class SurfaceFExtractor : public al::perception::FeatureExtractor
    {
    public:
      SurfaceFExtractor (ros::NodeHandle* nh);

      virtual
      ~SurfaceFExtractor ();

      void
      extract (al::perception::Entity &entity);

      void
      enableNormals (bool enable);

      void
      enableCurvatures (bool enable);

      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>*
      getNormalEstimator ();

    private:
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_;
      pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce_;
      pcl::PointCloud<pcl::PrincipalCurvatures> princip_curves_;

      bool enable_normals_;
      bool enable_curvatures_;
    };
  }
}

#endif /* SURFACEFEXTRACTOR_H_ */
