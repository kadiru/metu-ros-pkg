/*
 * adjective_predictor.cpp
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
 *     * Neither the name of KOVAN Lab nor the names of its
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

#include "al_srvs/GetLearnedAdjectives.h"
#include "al_srvs/GetLearnedEffects.h"

#include "al_utils/al_utils.h"
#include "al_estimation/SVMPredictor.h"

#include "map"

ros::NodeHandle* nh_;
std::vector<al::learning::SVMPredictor*> svm_predictors_;

std::map<int8_t, uint> map_behavior_id_to_f_order_;
std::map<int8_t, uint> map_effect_id_to_f_order_;

const int N_EFFECT_PER_BEHAVIOR = 8;

void
initSVMPredictors ();

bool
initSVMPredictor (const int adjective_type);

bool
    getLearnedAdjectivesCallback (al_srvs::GetLearnedAdjectives::Request &req,
                                  al_srvs::GetLearnedAdjectives::Response &res);

bool
getLearnedAdjectivesAllEffectsCallback (al_srvs::GetLearnedAdjectives::Request &req,
                                        al_srvs::GetLearnedAdjectives::Response &res);

bool
getScaleParameters (std::vector<al::learning::ScaleParams> &scale_parameters, const std::string scale_file_path);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_adjective_prediction");
  nh_ = new ros::NodeHandle ("~");

  ros::ServiceServer srv_get_learned_adjectives_;
  ros::ServiceServer srv_get_learned_adjectives_all_effects_;
  srv_get_learned_adjectives_ = nh_->advertiseService ("/get_learned_adjectives", &getLearnedAdjectivesCallback);
  srv_get_learned_adjectives_all_effects_ = nh_->advertiseService ("/get_learned_adjectives_all_effects",
                                                                   &getLearnedAdjectivesAllEffectsCallback);

  initSVMPredictors ();

  ros::Rate r (50);
  while (nh_->ok ())
  {
    ros::spinOnce ();
    r.sleep ();
  }

  return 0;
}

bool
initSVMPredictor (const int adjective_type)
{
  std::string directory_path;
  if (!nh_->getParam ("models_directory_path", directory_path))
    ROS_WARN("no such parameter as <models_directory_path>");

  bool use_full_effects = true;
  if (!nh_->getParam ("use_full_effects", use_full_effects))
    ROS_WARN("no such parameter as <use_full_effects>");

  std::string file_path;
  std::string model_name = al::facilities::adjTypeEnumToString (adjective_type);
  if (!model_name.length ())
  {
    ROS_WARN("EXITING!");
    exit (-1);
  }

  std::string param_name = "/svm_models/adjective/" + model_name;

  if (!nh_->getParam (param_name + "/model_file_path", file_path))
    ROS_WARN("no such parameter as <%s/model_file_path>", param_name.c_str());
  else
  {
    std::vector<int> indices;

    if (!use_full_effects)
    {

      //there is a model for this adjective, create an SVMPredictor
      indices.resize (9);
      //TODO: CAUTION: This is due to the fact that the relevancy indices start from 1, and it is used in this way.
      for (uint i = 0; i < indices.size (); i++)
        indices[i] = i + 1;
    }
    else
    {
      XmlRpc::XmlRpcValue list;
      if (!nh_->getParam (param_name + "/relevant_f_indices", list))
      {
        ROS_WARN("no such parameter as <%s/relevant_f_indices>", param_name.c_str());
        ROS_WARN("skipping svm model for the behavior %s", al::facilities::adjTypeEnumToString (adjective_type).c_str());
        return false;
      }
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for (int32_t j = 0; j < list.size (); j++)
      {
        ROS_ASSERT(list[j].getType() == XmlRpc::XmlRpcValue::TypeInt);
        indices.push_back (static_cast<int> (list[j]));
      }
      if (!indices.size ())
      {
        ROS_WARN("number of relevant features is zero!");
        ROS_WARN("skipping svm model for the behavior %s", al::facilities::adjTypeEnumToString (adjective_type).c_str());
        return false;
      }
    }

    std::vector<al::learning::ScaleParams> scale_params;

    if (!nh_->getParam (param_name + "/scale_file_path", file_path))
      ROS_WARN("no such parameter as <%s/scale_file_path>", param_name.c_str());
    else
    {
      if (!getScaleParameters (scale_params, directory_path + "/" + file_path))
        ROS_WARN("problem with scale parameters file!");
    }

    svm_predictors_.push_back (new al::learning::SVMPredictor (adjective_type, directory_path + "/" + file_path,
                                                               indices));
  }
  return true;
}

void
initSVMPredictors ()
{
  bool use_full_effects = true;
  if (!nh_->getParam ("use_full_effects", use_full_effects))
    ROS_WARN("no such parameter as <use_full_effects>");

  if (use_full_effects)
  {
    map_behavior_id_to_f_order_[al_msgs::Behavior::GRASP_TOP] = 0;
    map_behavior_id_to_f_order_[al_msgs::Behavior::GRASP_SIDE] = 1;
    map_behavior_id_to_f_order_[al_msgs::Behavior::PUSH_RGT] = 2;
    map_behavior_id_to_f_order_[al_msgs::Behavior::PUSH_LFT] = 3;
    map_behavior_id_to_f_order_[al_msgs::Behavior::PUSH_FWD] = 4;
    map_behavior_id_to_f_order_[al_msgs::Behavior::PUSH_BWD] = 5;

    map_effect_id_to_f_order_[al_msgs::Effect::MOVED_RIGHT] = 0;
    map_effect_id_to_f_order_[al_msgs::Effect::MOVED_LEFT] = 0;
    map_effect_id_to_f_order_[al_msgs::Effect::MOVED_FORWARD] = 0;
    map_effect_id_to_f_order_[al_msgs::Effect::MOVED_BACKWARD] = 0;
    map_effect_id_to_f_order_[al_msgs::Effect::TOPPLED] = 0;
    map_effect_id_to_f_order_[al_msgs::Effect::NO_CHANGE] = 0;
    map_effect_id_to_f_order_[al_msgs::Effect::GRASPED] = 0;
    map_effect_id_to_f_order_[al_msgs::Effect::DISAPPEARED] = 0;

  }

  initSVMPredictor (al::learning::adj_thin_thick);
  initSVMPredictor (al::learning::adj_round_edgy);
  initSVMPredictor (al::learning::adj_tall_short);
  initSVMPredictor (al::learning::adj_big_small);
}

bool
getScaleParameters (std::vector<al::learning::ScaleParams> &scale_parameters, const std::string scale_file_path)
{
  //read scale_file_path here
  std::ifstream in_file;
  in_file.open (scale_file_path.c_str (), std::ios::in);
  if (!in_file.is_open ())
  {
    ROS_ERROR("error reading scale file %s, EXITING!", scale_file_path.c_str());
    exit (-1);
  }
  else
  {
    al::learning::ScaleParams sp;
    float min, max;
    int line_number;
    char ch;

    in_file >> ch;
    std::cout << "Char is: " << ch << std::endl;

    in_file >> min >> max;
    std::cout << "Float numbers: " << min << ", " << max << std::endl;

    while (true)
    {
      in_file >> line_number;
      if (in_file.eof ())
        break;

      in_file >> min >> max;

      sp.min = min;
      sp.max = max;

      scale_parameters.push_back (sp);
    }
    for (uint i = 0; i < scale_parameters.size (); i++)
    {
      std::cout << scale_parameters[i].min << " " << scale_parameters[i].max << std::endl;
    }
  }
  return true;
}

bool
getLearnedAdjectivesCallback (al_srvs::GetLearnedAdjectives::Request &req, al_srvs::GetLearnedAdjectives::Response &res)
{
  al_srvs::GetLearnedEffects srv_effects;
  al_msgs::FeatureVector feature_vector;
  srv_effects.request.feature_vector = req.feature_vector;
  if (!ros::service::call ("/get_learned_effects", srv_effects))
  {
    ROS_WARN("effect prediction service call failed!");
    return false;
  }
  else
  {
    //instance vector: <REACHED><MOVED_LEFT><MOVED_RIGHT><MOVED_FORWARD><MOVED_BACKWARD><GRASPED><TOPPLED><VANISHED><NO_CHANGE<adjective>
    feature_vector.features.resize (9);
    for (uint i = 0; i < feature_vector.features.size (); i++)
    {
      feature_vector.features[i].type = al_msgs::Feature::AFFORDANCE_ACC;
      feature_vector.features[i].val_type = al_msgs::Feature::SINGLE_VALUED;
      feature_vector.features[i].avg = 0;
    }

    for (uint i = 0; i < srv_effects.response.effects.size (); i++)
    {
      switch (srv_effects.response.effects[i].effect)
      {
        case al_msgs::Effect::REACHED:
          if (srv_effects.response.effects[i].prob > feature_vector.features[0].avg)
            feature_vector.features[0].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::MOVED_LEFT:
          if (srv_effects.response.effects[i].prob > feature_vector.features[1].avg)
            feature_vector.features[1].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::MOVED_RIGHT:
          if (srv_effects.response.effects[i].prob > feature_vector.features[2].avg)
            feature_vector.features[2].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::MOVED_FORWARD:
          if (srv_effects.response.effects[i].prob > feature_vector.features[3].avg)
            feature_vector.features[3].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::MOVED_BACKWARD:
          if (srv_effects.response.effects[i].prob > feature_vector.features[4].avg)
            feature_vector.features[4].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::GRASPED:
          if (srv_effects.response.effects[i].prob > feature_vector.features[5].avg)
            feature_vector.features[5].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::TOPPLED:
          if (srv_effects.response.effects[i].prob > feature_vector.features[6].avg)
            feature_vector.features[6].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::VANISHED:
          if (srv_effects.response.effects[i].prob > feature_vector.features[7].avg)
            feature_vector.features[7].avg = srv_effects.response.effects[i].prob;
          break;
        case al_msgs::Effect::NO_CHANGE:
          if (srv_effects.response.effects[i].prob > feature_vector.features[8].avg)
            feature_vector.features[8].avg = srv_effects.response.effects[i].prob;
          break;
        default:
          ROS_WARN("check adjective feature vector, this effect wasn't included before");
          break;
      }
    }
    for (uint i = 0; i < svm_predictors_.size (); i++)
    {
      //TODO: this can be utilized later
      double most_likely_label = 0;

      std::cout << " --- " << std::endl;
      double* adj_probs = svm_predictors_[i]->getLabelPredictions (feature_vector, most_likely_label);

      std::cout << "most likely adjective: " << al::facilities::specEnumToString ((int8_t)most_likely_label)
          << std::endl;
      std::vector<int8_t> adj_ids = svm_predictors_[i]->getLabels ();

      for (uint j = 0; j < adj_ids.size (); j++)
      {
        al_msgs::Spec adj;

        adj.spec = adj_ids[j];
        adj.prob = (float)adj_probs[j];
        res.adjectives.push_back (adj);
      }
    }
  }

  return true;
}

bool
getLearnedAdjectivesAllEffectsCallback (al_srvs::GetLearnedAdjectives::Request &req,
                                        al_srvs::GetLearnedAdjectives::Response &res)
{
  al_srvs::GetLearnedEffects srv_effects;
  al_msgs::FeatureVector feature_vector;
  srv_effects.request.feature_vector = req.feature_vector;
  if (!ros::service::call ("/get_learned_effects", srv_effects))
  {
    ROS_WARN("effect prediction service call failed!");
    return false;
  }
  else
  {
    //instance vector: <REACHED><MOVED_LEFT><MOVED_RIGHT><MOVED_FORWARD><MOVED_BACKWARD><GRASPED><TOPPLED><VANISHED><NO_CHANGE<adjective>
    //for all behaviors <GRASP_TOP><GRASP_SIDE><PUSH_RGT><PUSH_LFT><PUSH_FWD><PUSH_BWD>
    feature_vector.features.resize (48);
    for (uint i = 0; i < feature_vector.features.size (); i++)
    {
      feature_vector.features[i].type = al_msgs::Feature::AFFORDANCE_ACC;
      feature_vector.features[i].val_type = al_msgs::Feature::SINGLE_VALUED;
      feature_vector.features[i].avg = 0;
    }

    for (uint i = 0; i < srv_effects.response.effects.size (); i++)
    {
      if (map_behavior_id_to_f_order_.find (srv_effects.response.effects[i].arg) == map_behavior_id_to_f_order_.end ())
      {
        ROS_WARN("unexpected behavior result obtained!");
        continue;
      }

      feature_vector.features[i].avg = map_behavior_id_to_f_order_[srv_effects.response.effects[i].arg]
          * N_EFFECT_PER_BEHAVIOR + map_effect_id_to_f_order_[srv_effects.response.effects[i].effect];
    }

    for (uint i = 0; i < svm_predictors_.size (); i++)
    {
      //TODO: this can be utilized later
      double most_likely_label = 0;

      std::cout << " --- " << std::endl;
      double* adj_probs = svm_predictors_[i]->getLabelPredictions (feature_vector, most_likely_label);

      std::cout << "most likely adjective: " << al::facilities::specEnumToString ((int8_t)most_likely_label)
          << std::endl;
      std::vector<int8_t> adj_ids = svm_predictors_[i]->getLabels ();

      for (uint j = 0; j < adj_ids.size (); j++)
      {
        al_msgs::Spec adj;

        adj.spec = adj_ids[j];
        adj.prob = (float)adj_probs[j];
        res.adjectives.push_back (adj);
      }
    }
  }
  return true;
}
