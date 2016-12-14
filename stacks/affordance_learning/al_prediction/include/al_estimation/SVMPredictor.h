/*
 * SVMPredictor.h
 * Copyright (c) 2012, Kadir Firat Uyanik, Kovan Research Lab, METU
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
 *     * Neither the name of Willow Garage nor the names of its
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

#ifndef SVMPREDICTOR_H_
#define SVMPREDICTOR_H_

#include "libsvm/svm.h"

#include "al_msgs/FeatureVector.h"

namespace al
{
  namespace learning
  {
    struct ScaleParams
    {
      float min;
      float max;
      float y_min;
      float y_max;
    };

    class SVMPredictor
    {
    public:

      SVMPredictor (const int relation, const std::string &model_file_path,
                    const std::vector<int> &relevant_feature_indices);

      virtual
      ~SVMPredictor ();

      double*
      getLabelPredictions (const al_msgs::FeatureVector& feature_vector, double& predict_label);

      std::vector<int8_t>
      getLabels ();

      int
      getRelation ();

      bool
      setScaleParameters (std::vector<ScaleParams> scale_parameters);

      svm_model* model_;
    private:
      //      svm_model* model_;
      svm_node* node_;
      std::vector<int> relevant_f_indices_;
      std::vector<ScaleParams> scale_parameters_;
      std::vector<int8_t> labels_;
      double* p_labels_;
      int relation_;
    };
  }
}

#endif /* SVMPREDICTOR_H_ */
