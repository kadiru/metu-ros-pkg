/*
 * FeatureExtractor.cpp
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

#include "al_perception/feature_extractors/FeatureExtractor.h"

namespace al
{

  namespace perception
  {

    FeatureExtractor::FeatureExtractor (ros::NodeHandle* nh)
    {
      nh_ = nh;
    }

    bool
    FeatureExtractor::calcFeature (al_msgs::Feature &feature, cv::Mat feature_data)
    {
      cv::MatND feature_hist = cv::MatND (1, feature.n_hist_bins, CV_16U);
      float range[] = {feature.range_min, feature.range_max};
      const float* ranges[] = {range};
      const int channel = 0;
      const int n_bins = feature.n_hist_bins;
      cv::calcHist (&feature_data, 1, &channel, cv::Mat (), feature_hist, 1, &n_bins, ranges, true, false);
      double min = 0;
      double max = 0;
      cv::Scalar avg = 0;
      cv::Scalar std_dev = 0;
      cv::minMaxLoc (feature_data, &min, &max, 0, 0);
      cv::meanStdDev (feature_data, avg, std_dev);
      feature.min = (float)min;
      feature.max = (float)max;
      //      feature.avg = (avg.val[0] - feature.range_min) / (feature.range_max - feature.range_min);
      //      feature.dev = (std_dev.val[0] - feature.range_min) / (feature.range_max - feature.range_min);
      feature.avg = avg.val[0];
      feature.dev = std_dev.val[0];
      feature.var = feature.dev * feature.dev;

      feature.his.resize (feature.n_hist_bins);
      for (uint16_t ci = 0; ci < feature.n_hist_bins; ci++)
        feature.his[ci] = ((float*)feature_hist.data)[ci];
    }

    FeatureExtractor::~FeatureExtractor ()
    {
      // TODO Auto-generated destructor stub
    }
  }

}
