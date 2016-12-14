/*
 * SVMPredictor.cpp
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

#include "../include/al_estimation/SVMPredictor.h"

namespace al
{

  namespace learning
  {

    SVMPredictor::SVMPredictor (const int relation, const std::string &model_file_path,
                                const std::vector<int> &relevant_feature_indices)
    {
      relation_ = relation;
      relevant_f_indices_ = relevant_feature_indices;
      std::cout << model_file_path << std::endl;
      model_ = svm_load_model (model_file_path.c_str ());
      node_ = new svm_node[relevant_f_indices_.size () + 1];
      p_labels_ = new double[model_->nr_class];
      std::cout << (int)model_->nr_class << std::endl;
      labels_.resize ((uint)model_->nr_class);
      for (uint i = 0; i < (uint)model_->nr_class; i++)
        labels_[i] = (int8_t)model_->label[i];

    }

    SVMPredictor::~SVMPredictor ()
    {
      // TODO Auto-generated destructor stub
    }

    std::vector<int8_t>
    SVMPredictor::getLabels ()
    {
      return labels_;
    }

    bool
    SVMPredictor::setScaleParameters (std::vector<ScaleParams> scale_parameters)
    {
      scale_parameters_ = scale_parameters;
      return true;
    }

    double*
    SVMPredictor::getLabelPredictions (const al_msgs::FeatureVector& feature_vector, double& predict_label)
    {
      std::cout << "1" << std::endl;
      //first get raw feature vector
      //TODO: push_back is not efficient, do this with resizing the vector at the very first
      std::vector<float> raw_feature_vector;
      std::vector<float> selected_feature_vector;
      for (uint i = 0; i < feature_vector.features.size (); i++)
      {
        std::cout << "2" << std::endl;
        if (feature_vector.features[i].val_type == al_msgs::Feature::SINGLE_VALUED)
        {
          std::cout << "3" << std::endl;
          raw_feature_vector.push_back (feature_vector.features[i].avg);
          //          std::cout << "Single valued feat." << i << " = " << (float)(feature_vector.features[i].avg) << std::endl;
          continue;
        }
        else
        {
          std::cout << "4" << std::endl;
          raw_feature_vector.push_back (feature_vector.features[i].min);
          raw_feature_vector.push_back (feature_vector.features[i].max);
          raw_feature_vector.push_back (feature_vector.features[i].avg);
          raw_feature_vector.push_back (feature_vector.features[i].dev);
          raw_feature_vector.push_back (feature_vector.features[i].var);

          for (uint j = 0; j < feature_vector.features[i].his.size (); j++)
            raw_feature_vector.push_back (feature_vector.features[i].his[j]);
        }
      }

      std::cout << "5" << std::endl;
      for (uint i = 0; i < relevant_f_indices_.size (); i++)
      {
        try
        {
          node_[i].index = (i + 1);
          node_[i].value = raw_feature_vector[relevant_f_indices_[i] - 1];

          //by default scale to [-1,1], this is "2" below
          if (scale_parameters_.size ())
          {
            //            node_[i].value = scale_parameters_[i].min + (2) * (node_[i].value - scale_parameters_[i].min)
            //                / (scale_parameters_[i].max - scale_parameters_[i].min);

            node_[i].value = -1 + (2) * (node_[i].value - scale_parameters_[i].min) / (scale_parameters_[i].max
                - scale_parameters_[i].min);
          }
          else
            ROS_WARN("no scaling done for this learning task");

          //          std::cout << "Inserted feature name " << " index: " << relevant_f_indices_[i] - 1 << " value: "
          //              << node_[i].value << std::endl;
        }
        catch (std::exception e)
        {
          std::cout << e.what () << std::endl;
        }
      }

      node_[relevant_f_indices_.size ()].index = -1;
      double* prob = new double[model_->nr_class];
      //      std::cout << " ------- " << std::endl;
      //      std::cout << " ------- " << std::endl;
      //      std::cout << " ------- " << std::endl;
      //      std::cout << " ------- " << std::endl;
      //      std::cout << " ------- " << std::endl;
      //      std::cout << " ------- " << std::endl;
      //      std::cout << "#classes : " << model_->nr_class << std::endl;
      //      std::cout << "#svs     : " << model_->l << std::endl;
      //      for (uint i = 0; i < model_->nr_class; i++)
      //        for (uint j = 0; j < model_->l; j++)
      //          std::cout << "#svs     : " << model_->sv_coef[i][j] << std::endl;
      //      std::cout << " ------- " << std::endl;

      predict_label = svm_predict_probability (model_, node_, p_labels_);
      //      std::cout << "most likely: " << predict_label << std::endl;
      //      for (uint i = 0; i < model_->nr_class; i++)
      //        std::cout << p_labels_[i] << std::endl;

      return p_labels_;
    }

    int
    SVMPredictor::getRelation ()
    {
      return relation_;
    }
  }
}
