/*
 * object_shape_predictor.cpp
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

#include "al_srvs/GetLearnedObjectShapes.h"

#include "al_utils/al_utils.h"
#include "al_estimation/SVMPredictor.h"

#include "fstream"

ros::NodeHandle* nh_;
std::vector<al::learning::SVMPredictor*> svm_predictors_;

bool
getLearnedObjectShapesCallback (al_srvs::GetLearnedObjectShapes::Request &req,
                                al_srvs::GetLearnedObjectShapes::Response &res);

void
initSVMPredictors ();

al::learning::SVMPredictor*
getSVMPredictorByRelation (int8_t relation);

bool
getShapeEstimates (const al_msgs::FeatureVector& feature_vector, std::vector<al_msgs::Shape>& shapes);

bool
getScaleParameters (std::vector<al::learning::ScaleParams> &scale_parameters, const std::string scale_file_path);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_shape_prediction");
  nh_ = new ros::NodeHandle ("~");
  ros::ServiceServer srv_get_learned_object_shape_;
  srv_get_learned_object_shape_ = nh_->advertiseService ("/get_learned_object_shapes", &getLearnedObjectShapesCallback);

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

  std::string param_name = std::string ("/svm_models/object/") + std::string ("SHAPE");

  if (!nh_->getParam (param_name + "/model_file_path", file_path))
    ROS_WARN("no such parameter as <%s/model_file_path>", param_name.c_str());
  else
  {
    //there is a model for this relation, create an SVMPredictor if relevant features are given correctly
    std::vector<int> indices;
    std::vector<al::learning::ScaleParams> scale_params;
    XmlRpc::XmlRpcValue list;
    if (!nh_->getParam (param_name + "/relevant_f_indices", list))
    {
      ROS_WARN("no such parameter as <%s/relevant_f_indices>", param_name.c_str());
      ROS_WARN("skipping svm model for SHAPE");
      return;
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
      ROS_WARN("skipping svm model for SHAPE");
      return;
    }
    svm_predictors_.push_back (new al::learning::SVMPredictor (0, directory_path + "/" + file_path, indices));

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

bool
getLearnedObjectShapesCallback (al_srvs::GetLearnedObjectShapes::Request &req,
                                al_srvs::GetLearnedObjectShapes::Response &res)
{
  //  std::cout << "service call received\n";
  //  //  std::cout << req.feature_vector << std::endl;
  //  return getShapeEstimates (req.feature_vector, res.shapes);

  std::cout << "service call received\n";

  std::map<float, al_msgs::Shape> shapes_sorted_by_prob;
  std::map<float, al_msgs::Shape>::iterator it;

  if (getShapeEstimates (req.feature_vector, res.shapes))
  {
    for (uint i = 0; i < res.shapes.size (); i++)
    {
      al_msgs::Shape shape;
      shape = res.shapes[i];
      shapes_sorted_by_prob[shape.prob] = shape;
    }
    res.shapes.resize (shapes_sorted_by_prob.size ());
    int cnt = 0;
    for (it = shapes_sorted_by_prob.begin (); it != shapes_sorted_by_prob.end (); it++)
    {
      res.shapes[cnt] = it->second;
      cnt++;
    }
    std::cout << "** ** **" << std::endl;
    for (uint i = 0; i < res.shapes.size (); i++)
      std::cout << al::facilities::shapeEnumToString (res.shapes[i].shape) << "\t" << res.shapes[i].prob << std::endl;
    std::cout << "-- -- --" << std::endl;
    return true;
  }
  else
    return false;
}

bool
getShapeEstimates (const al_msgs::FeatureVector& feature_vector, std::vector<al_msgs::Shape>& shapes)
{
  std::cout << "***** update *****" << std::endl;
  shapes.clear ();
  for (uint i = 0; i < svm_predictors_.size (); i++)
  {
    //TODO: this can be utilized later
    double most_likely_label = 0;

    std::cout << " --- " << std::endl;
    double* shape_probs = svm_predictors_[i]->getLabelPredictions (feature_vector, most_likely_label);
    //    for (uint j = 0; j < svm_predictors_[i]->model_->nr_class; j++)
    //      std::cout << shape_probs[j] << std::endl;

    std::vector<int8_t> shape_ids = svm_predictors_[i]->getLabels ();
    std::cout << "most likely shape: " << al::facilities::shapeEnumToString ((int8_t)most_likely_label) << std::endl;

    for (uint j = 0; j < shape_ids.size (); j++)
    {
      al_msgs::Shape shape;
      shape.shape = shape_ids[j];
      shape.prob = (float)shape_probs[j];
      shapes.push_back (shape);
    }

    //    al_msgs::Shape shape;
    //    shape.shape = most_likely_label;
    //      shape.prob = (float)shape_probs[j];
    //    shapes.push_back (shape);
  }
  for (uint i = 0; i < shapes.size (); i++)
  {
    std::cout << al::facilities::shapeEnumToString (shapes[i].shape) << "\t";
    std::cout << shapes[i].prob << std::endl;
  }
  return true;
}
