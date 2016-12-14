/*
 * viz_objects_features.cpp
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

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

//TODO: import this header properly
#include "/usr/local/MATLAB/R2011a/extern/include/engine.h"

#include "al_msgs/CloudObjects.h"
#include "al_msgs/Entities.h"
#include "al_srvs/GetEntitiesVisualFeatures.h"

#include "ros/ros.h"
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d_omp.h>

ros::NodeHandle* nh_;
ros::Subscriber sub_entities_;
ros::ServiceClient srv_cl_get_entities_visual_features_;
al_msgs::Entities entities_;
Engine* ep_;

pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_;
pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce_;
pcl::PointCloud<pcl::PrincipalCurvatures> princip_curves_;

bool msg_entities_rcvd_ = false;

void
entitiesCallback (al_msgs::EntitiesConstPtr entities);

//first obtains the features then visualize them
bool
vizFeatures ();

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_viz_objects_features");
  nh_ = new ros::NodeHandle ("~");

  sub_entities_ = nh_->subscribe ("/perception/entities", 1, &entitiesCallback);
  srv_cl_get_entities_visual_features_
      = nh_->serviceClient<al_srvs::GetEntitiesVisualFeatures> ("/get_entities_visual_features", true);
  srv_cl_get_entities_visual_features_.waitForExistence ();
  srv_cl_get_entities_visual_features_
      = nh_->serviceClient<al_srvs::GetEntitiesVisualFeatures> ("/get_entities_visual_features", true);

  ep_ = engOpen ("matlab -nodesktop -nosplash -nojvm");
  if (ep_ == NULL)
    ROS_ERROR("Constructor: engOpen failed");
  else
    ROS_DEBUG(" Matlab engine is working!");

  //in any case surface normals should be calculated
  pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  ne_.setSearchMethod (tree);
  ne_.setViewPoint (0, 0, 1.2);//this could be done more informed by getting this information from a topic

  ros::Rate r (30);//objects can be refreshed at most @30hz
  while (nh_->ok ())
  {
    if (msg_entities_rcvd_)
    {
      msg_entities_rcvd_ = false;
      vizFeatures ();
    }
    ros::spinOnce ();
    r.sleep ();
  }
  engClose (ep_);

  return 0;
}

bool
vizFeatures ()
{
  //visualize the features, it is guaranteed that there is no problem in the feature extraction
  if (ep_ != NULL)
  {
    al_srvs::GetEntitiesVisualFeatures::Request req;
    req.entities = entities_;
    al_srvs::GetEntitiesVisualFeatures::Response res;

    if (!srv_cl_get_entities_visual_features_.call (req, res))
    {
      //TODO: this can be handled better
      ROS_WARN("entities feature calculation service call failed!");
      return false;
    }
    //if there is no object, plot will have 0 row and 0 col, this might be a problem in matlab
    int plot_n_row = 0;
    int plot_n_col = 0;
    for (uint i = 0; i < res.feature_vectors.size (); i++)
    {
      if (res.feature_vectors[i].features.size ())
      {
        plot_n_row++;
        int n_col = 0;
        for (uint j = 0; j < res.feature_vectors[i].features.size (); j++)
        {
          if (res.feature_vectors[i].features[j].val_type == al_msgs::Feature::DISPERSIVE_VALUED)
            n_col++;
          if (n_col > plot_n_col)
            plot_n_col = n_col;
        }
        //          if (plot_n_col < (int)res.feature_vectors[i].features.size ())
        //            plot_n_col = res.feature_vectors[i].features.size ();
      }
    }

    if (!plot_n_row || !plot_n_col)
      return false;

    //    std::cout << "plot_n_row: " << plot_n_row << "\tplot_n_col: " << plot_n_col << std::endl;

    int row_index = -1;
    for (uint8_t i = 0; i < req.entities.entities.size (); i++)
    {
      if (!res.feature_vectors[i].features.size ())
        continue;

      row_index++;
      al_msgs::FeatureVector feature_vector = res.feature_vectors[i];
      for (uint8_t j = 0; j < feature_vector.features.size (); j++)
      {
        //is it a histogramable feature ?
        if (feature_vector.features[j].val_type == al_msgs::Feature::DISPERSIVE_VALUED)
        {
          //          std::cout << feature_vector.features[j].type << std::endl;
          //          for (uint8_t u = 0; u < feature_vector.features[j].n_hist_bins; u++)
          //            std::cout << feature_vector.features[j].his[u] << " ";
          //          std::cout << std::endl;

          mxArray *x_axial = mxCreateNumericMatrix ((int)feature_vector.features[j].n_hist_bins, 1, mxDOUBLE_CLASS,
                                                    mxREAL);
          mxArray *y_axial = mxCreateNumericMatrix ((int)feature_vector.features[j].n_hist_bins, 1, mxDOUBLE_CLASS,
                                                    mxREAL);

          float x_axial_data_step_size = (feature_vector.features[j].range_max - feature_vector.features[j].range_min)
              / (int)feature_vector.features[j].n_hist_bins;

          std::vector<double> x_axial_data ((int)feature_vector.features[j].his.size ());
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
          s << "subplot(" << plot_n_row << "," << plot_n_col << "," << row_index * plot_n_col + j + 1 << ")";
          std::string cmd = s.str ();
          ROS_DEBUG("%s", cmd.c_str());

          std::stringstream title_s;
          title_s << "title('" << "Object: " << req.entities.entities[i].collision_object.id << " "
              << feature_vector.features[j].type << "')";
          std::string title_cmd = title_s.str ();
          ROS_DEBUG("%s", title_cmd.c_str());

          std::string bar_cmd = "bar(" + (feature_vector.features[j].type + "_x,") + feature_vector.features[j].type
              + "_y" + ",'histc');";
          ROS_DEBUG("%s", bar_cmd.c_str());

          std::string make_up_cmd = "set(gca,'XLim',[";//0 360]);";
          if (feature_vector.features[j].type == al_msgs::Feature::CURV_MAX || feature_vector.features[j].type
              == al_msgs::Feature::CURV_MIN)
          {
            make_up_cmd += "0 1])";
          }
          else if (feature_vector.features[j].type == al_msgs::Feature::SHAPE_INDEX)
          {
            make_up_cmd += "-1 1])";
          }
          else
          {
            make_up_cmd += "0 360])";
          }
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
  }
  return true;
}

void
entitiesCallback (al_msgs::EntitiesConstPtr entities)
{
  msg_entities_rcvd_ = true;
  entities_ = *entities;
}
