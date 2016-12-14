/*
 * viz_object_features.cpp
 * Copyright (c) 2012, Kadir Firat Uyanik, KOVAN Research Lab, METU
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

#include "/usr/local/MATLAB/R2012a/extern/include/engine.h"

#include "al_utils/al_utils.h"
#include "al_srvs/GetEntityVisualFeatures.h"
#include "al_srvs/GetEntity.h"

#include "ros/ros.h"

ros::NodeHandle* nh_;
ros::ServiceClient srv_cl_get_entity_;
ros::ServiceClient srv_cl_get_entity_visual_features_;
Engine* ep_;
//bool msg_entity_rcvd_ = false;

//void
//entityCallback (al_msgs::EntityConstPtr entity);

//first obtains the features then visualize them
bool
vizFeatures (const std::string id, const bool latest_scene_required = true);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_viz_objects_features");
  nh_ = new ros::NodeHandle ("~");

  srv_cl_get_entity_ = nh_->serviceClient<al_srvs::GetEntity> ("/memory/get_entity", true);

  srv_cl_get_entity_visual_features_
      = nh_->serviceClient<al_srvs::GetEntityVisualFeatures> ("/get_entity_visual_features", true);

  srv_cl_get_entity_visual_features_.waitForExistence ();
  srv_cl_get_entity_visual_features_
      = nh_->serviceClient<al_srvs::GetEntityVisualFeatures> ("/get_entity_visual_features", true);

  ep_ = engOpen ("matlab -nodesktop -nosplash -nojvm");
  if (!(ep_ = engOpen (NULL)))
  {
    printf ("Can't start MATLAB engine");
    exit (-1);
  }

  //  ep_ = engOpen ("matlab -nodesktop");
  if (ep_ == NULL)
    ROS_ERROR("Constructor: engOpen failed");
  else
    ROS_INFO(" Matlab engine is working!");

  std::cout << "Enter an object id to visualize its features!" << std::endl;
  bool request_rcvd = false;
  std::string req_id;
  bool latest_scene_required = true;
  ros::Rate r (30);//objects can be refreshed at most @30hz
  while (nh_->ok ())
  {
    char c_id;
    if (al::system::getKey (c_id))
    {
      //      req_id = std::string (&c_id);
      req_id = std::string (&c_id);
      std::cout << "requested id: " << req_id << std::endl;
      std::cout << "is latest scene required?" << std::endl;
      std::cin >> c_id;
      if (!atoi (&c_id))
        latest_scene_required = false;
      else
        latest_scene_required = true;

      request_rcvd = true;
      std::cout << "Enter an object id to visualize its features!" << std::endl;
    }

    std::string s ("5");
    if (request_rcvd)
      //      vizFeatures (req_id, latest_scene_required);
      vizFeatures (s, true);
    ros::spinOnce ();
    r.sleep ();
  }
  engClose (ep_);

  return 0;
}

bool
vizFeatures (const std::string id, const bool latest_scene_required)
{
  //visualize the features, it is guaranteed that there is no problem in the feature extraction
  if (ep_ != NULL)
  {
    al_srvs::GetEntity::Request req_entity;
    al_srvs::GetEntity::Response res_entity;
    req_entity.id = id;
    req_entity.latest_scene_required = latest_scene_required;
    if (!srv_cl_get_entity_.call (req_entity, res_entity))
    {
      //TODO: this can be handled better
      ROS_WARN("entity  service call failed!");
      return false;
    }

    al_srvs::GetEntityVisualFeatures::Request req;
    al_srvs::GetEntityVisualFeatures::Response res;
    req.entity = res_entity.entity;

    if (!srv_cl_get_entity_visual_features_.call (req, res))
    {
      //TODO: this can be handled better
      ROS_WARN("entity feature calculation service call failed!");
      return false;
    }

    if (!res.feature_vector.features.size ())
    {
      ROS_WARN("entity has no features!");
      return false;
    }

    al_msgs::FeatureVector feature_vector = res.feature_vector;
    for (uint8_t j = 0; j < feature_vector.features.size (); j++)
    {
      //is it a histogramable feature ?
      if (feature_vector.features[j].val_type != al_msgs::Feature::SINGLE_VALUED)
      {
        //          std::cout << feature_vector.features[j].type << std::endl;
        //          for (uint8_t u = 0; u < feature_vector.features[j].n_hist_bins; u++)
        //            std::cout << feature_vector.features[j].his[u] << " ";
        //          std::cout << std::endl;

        mxArray *x_axial = mxCreateNumericMatrix ((int)feature_vector.features[j].his.size (), 1, mxDOUBLE_CLASS,
                                                  mxREAL);
        mxArray *y_axial = mxCreateNumericMatrix ((int)feature_vector.features[j].his.size (), 1, mxDOUBLE_CLASS,
                                                  mxREAL);

        float x_axial_data_step_size = (feature_vector.features[j].range_max - feature_vector.features[j].range_min)
            / (int)feature_vector.features[j].n_hist_bins;

        std::vector<double> x_axial_data ((int)feature_vector.features[j].n_hist_bins);
        for (uint8_t k = 0; k < x_axial_data.size (); k++)
          x_axial_data[k] = feature_vector.features[j].range_min + (int)k * x_axial_data_step_size;

        std::vector<double> y_axial_data ((int)feature_vector.features[j].n_hist_bins);
        for (uint8_t k = 0; k < feature_vector.features[j].n_hist_bins; k++)
          y_axial_data[k] = (double)feature_vector.features[j].his[k];

        memcpy ((char *)mxGetPr (x_axial), reinterpret_cast<char*> (&x_axial_data[0]), (int)x_axial_data.size ()
            * sizeof(double));

        memcpy ((char *)mxGetPr (y_axial), reinterpret_cast<char*> (&y_axial_data[0]), (int)x_axial_data.size ()
            * sizeof(double));

        engPutVariable (ep_, (feature_vector.features[j].type + "_x").c_str (), x_axial);
        engPutVariable (ep_, (feature_vector.features[j].type + "_y").c_str (), y_axial);

        std::stringstream s;//create plot for this particular object
        s << "subplot(" << "1," << (int)feature_vector.features.size () << "," << j + 1 << ")";
        std::string cmd = s.str ();
        ROS_DEBUG("%s", cmd.c_str());

        std::stringstream title_s;
        title_s << "title('" << "Object: " << req.entity.collision_object.id << " " << feature_vector.features[j].type
            << "')";
        std::string title_cmd = title_s.str ();
        ROS_DEBUG("%s", title_cmd.c_str());

        std::string bar_cmd = "bar(" + (feature_vector.features[j].type + "_x,") + feature_vector.features[j].type
            + "_y" + ",'histc');";
        ROS_DEBUG("%s", bar_cmd.c_str());

        std::string make_up_cmd = "set(gca,'XLim',[0 360]);";
        std::string x_label_cmd = "xlabel(gca,'deg');";
        std::string y_label_cmd = "ylabel(gca,'#points');";

        int b = engEvalString (ep_, cmd.c_str ());
        b = engEvalString (ep_, bar_cmd.c_str ());
        b = engEvalString (ep_, make_up_cmd.c_str ());
        b = engEvalString (ep_, x_label_cmd.c_str ());
        b = engEvalString (ep_, y_label_cmd.c_str ());
        b = engEvalString (ep_, title_cmd.c_str ());
      }
    }
  }

  return true;
}

//void
//entityCallback (al_msgs::EntityConstPtr entity)
//{
//  msg_entity_rcvd_ = true;
//  entity_ = *entity;
//}
