/*
 * main.cpp
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

#include "al_srvs/GetLearnedEffects.h"
#include "al_srvs/GetLearnedEffect.h"
#include "al_srvs/GetBehaviorsByEffect.h"

#include "al_utils/al_utils.h"
#include "al_estimation/SVMPredictor.h"

#include "fstream"

ros::NodeHandle* nh_;
std::vector<al::learning::SVMPredictor*> svm_predictors_;
bool behaviorQueryMode;
int requestedEffect; // only used if a behavior is queried.

bool
getLearnedEffectsCallback (al_srvs::GetLearnedEffects::Request &req, al_srvs::GetLearnedEffects::Response &res);

bool
getLearnedEffectCallback (al_srvs::GetLearnedEffect::Request &req, al_srvs::GetLearnedEffect::Response &res);

bool
    getBehaviorsByEffectCallback (al_srvs::GetBehaviorsByEffect::Request &req,
                                  al_srvs::GetBehaviorsByEffect::Response &res);

void
initSVMPredictors ();

al::learning::SVMPredictor*
getSVMPredictorByRelation (int8_t relation);

bool
getEffectEstimates (const al_msgs::FeatureVector& feature_vector, std::vector<al_msgs::Effect>& effects);

bool
getScaleParameters (std::vector<al::learning::ScaleParams> &scale_parameters, const std::string scale_file_path);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_effect_prediction");
  nh_ = new ros::NodeHandle ("~");
  behaviorQueryMode = false;
  ros::ServiceServer srv_get_learned_effects_;
  ros::ServiceServer srv_get_learned_effect_;
  ros::ServiceServer srv_get_behaviors_by_effect_;
  srv_get_learned_effects_ = nh_->advertiseService ("/get_learned_effects", &getLearnedEffectsCallback);
  srv_get_learned_effect_ = nh_->advertiseService ("/get_learned_effect", &getLearnedEffectCallback);
  srv_get_behaviors_by_effect_ = nh_->advertiseService ("/get_behaviors_by_effect", &getBehaviorsByEffectCallback);

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

al::learning::SVMPredictor*
getSVMPredictorByRelation (int8_t relation)
{
  for (uint i = 0; i < svm_predictors_.size (); i++)
  {
    if (svm_predictors_[i]->getRelation () == relation)
      return svm_predictors_[i];
  }
  return NULL;
}

void
initSVMPredictors ()
{
  std::string directory_path;
  if (!nh_->getParam ("models_directory_path", directory_path))
    ROS_WARN("no such parameter as <models_directory_path>");

  std::string file_path;

  for (int8_t i = 0; i < al_msgs::Behavior::MAX_BEHAVIOR_INDEX; i++)
  {
    std::string param_name = "/svm_models/behavior/" + al::facilities::behaviorEnumToString (i);

    if (!nh_->getParam (param_name + "/model_file_path", file_path))
      ROS_WARN("no such parameter as <%s/model_file_path>", param_name.c_str());
    else
    {
      //there is a model for this behavior, create an SVMPredictor if relevant features are given correctly
      std::vector<int> indices;
      std::vector<std::string> effects;
      std::vector<al::learning::ScaleParams> scale_params;
      XmlRpc::XmlRpcValue list;
      if (!nh_->getParam (param_name + "/relevant_f_indices", list))
      {
        ROS_WARN("no such parameter as <%s/relevant_f_indices>", param_name.c_str());
        ROS_WARN("skipping svm model for the behavior %s", al::facilities::behaviorEnumToString (i).c_str());
        continue;
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
        ROS_WARN("skipping svm model for the behavior %s", al::facilities::behaviorEnumToString (i).c_str());
        continue;
      }
      svm_predictors_.push_back (new al::learning::SVMPredictor (i, directory_path + "/" + file_path, indices));

      if (!nh_->getParam (param_name + "/scale_file_path", file_path))
        ROS_WARN("no such parameter as <%s/scale_file_path>", param_name.c_str());
      else
      {
        if (!getScaleParameters (scale_params, directory_path + "/" + file_path))
          ROS_WARN("problem with scale parameters file!");
      }

      //      svm_predictors_.back ()->setScaleParameters (scale_params);
      svm_predictors_[svm_predictors_.size () - 1]->setScaleParameters (scale_params);
    }
  }
}

bool
getBehaviorsByEffectCallback (al_srvs::GetBehaviorsByEffect::Request &req, al_srvs::GetBehaviorsByEffect::Response &res)
{

//  behaviorQueryMode = true;
  requestedEffect = (int)req.effect_id;
  std::vector<al_msgs::Effect> effects;
  std::map<float, al_msgs::Behavior> behaviors_sorted_by_prob;
  std::map<float, al_msgs::Behavior>::iterator it;

  if (getEffectEstimates (req.entity.feature_vector, effects))
  {
    for (uint i = 0; i < effects.size (); i++)
    {
      if (effects[i].effect == req.effect_id)
      {
        al_msgs::Behavior behavior;
        behavior.behavior = effects[i].arg;
        behavior.entity = req.entity.collision_object.id;
        behavior.prob = effects[i].prob;
        behaviors_sorted_by_prob[behavior.prob] = behavior;
      }
    }
    res.behaviors.resize (behaviors_sorted_by_prob.size ());
    int cnt = 0;
    for (it = behaviors_sorted_by_prob.begin (); it != behaviors_sorted_by_prob.end (); it++)
    {
      res.behaviors[cnt] = it->second;
      cnt++;
    }
    //    for (uint i = 0; i < res.behaviors.size (); i++)
    //      std::cout << res.behaviors[i] << std::endl;
//    behaviorQueryMode = false;
    return true;
  }
  else
  {
    behaviorQueryMode = false;
    return false;
  }
}

bool
getLearnedEffectsCallback (al_srvs::GetLearnedEffects::Request &req, al_srvs::GetLearnedEffects::Response &res)
{
  std::cout << "service call received\n";
  return getEffectEstimates (req.feature_vector, res.effects);
}

bool
getLearnedEffectCallback (al_srvs::GetLearnedEffect::Request &req, al_srvs::GetLearnedEffect::Response &res)
{
  al::learning::SVMPredictor* svm_predictor = getSVMPredictorByRelation (req.behavior_id);

  //  std::cout << "Requesting affordances for behavior " << (int)req.behavior_id << std::endl;
  if (svm_predictor == NULL)
  {
    ROS_WARN("%s has not been learned yet!", al::facilities::behaviorEnumToString(req.behavior_id).c_str());
    return false;
  }
  else
  {
    double dummy_label;
    double* effect_probs = svm_predictor->getLabelPredictions (req.feature_vector, dummy_label);
    std::vector<int8_t> labels = svm_predictor->getLabels ();
    //    for (uint i = 0; i < labels.size (); i++)
    //    {
    //      al_msgs::Effect effect;
    //      effect.effect = labels[i];
    //      effect.prob = effect_probs[i];
    //      effect.arg = svm_predictor->getRelation ();
    //      res.effects.push_back (effect);
    //    }
    al_msgs::Effect effect;
    effect.effect = dummy_label;
    //      effect.prob = effect_probs[i];
    effect.arg = svm_predictor->getRelation ();
    res.effects.push_back (effect);

    //    for (uint i = 0; i < res.effects.size (); i++)
    //    {
    //      std::cout << al::facilities::behaviorEnumToString (res.effects[i].arg) << "\t";
    //      std::cout << al::facilities::effectEnumToString (res.effects[i].effect) << "\t";
    //      std::cout << res.effects[i].prob << std::endl;
    //    }
    return true;
  }
}

bool
getEffectEstimates (const al_msgs::FeatureVector& feature_vector, std::vector<al_msgs::Effect>& effects)
{
  std::cout << "***** update *****" << std::endl;
  effects.clear ();
  for (uint i = 0; i < svm_predictors_.size (); i++)
  {
    //TODO: this can be utilized later
    double most_likely_label = 0;

    std::cout << " --- " << std::endl;
    double* effect_probs = svm_predictors_[i]->getLabelPredictions (feature_vector, most_likely_label);

    std::cout << "behavior: " << al::facilities::behaviorEnumToString ((int8_t)svm_predictors_[i]->getRelation ())
        << "\tmost likely effect: " << al::facilities::effectEnumToString ((int8_t)most_likely_label) << std::endl;
    std::vector<int8_t> effect_ids = svm_predictors_[i]->getLabels ();

    for (uint j = 0; j < effect_ids.size (); j++)
    {
      al_msgs::Effect effect;

      effect.effect = effect_ids[j];
      effect.prob = (float)effect_probs[j];
      effect.arg = svm_predictors_[i]->getRelation ();//stores the behavior id
      effects.push_back (effect);

      //      if (behaviorQueryMode && ((int)effect_ids[j] == requestedEffect))
      //      {
      //        effect.effect = effect_ids[j];
      //        effect.prob = (float)effect_probs[j];
      //        effect.arg = svm_predictors_[i]->getRelation ();//stores the behavior id
      //        effects.push_back (effect);
      //      }
    }
  }
  for (uint i = 0; i < effects.size (); i++)
  {
    std::cout << al::facilities::behaviorEnumToString (effects[i].arg) << "\t";
    std::cout << al::facilities::effectEnumToString (effects[i].effect) << "\t";
    std::cout << effects[i].prob << std::endl;
  }
  return true;
}
