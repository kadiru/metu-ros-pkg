/*
 * al_utils.cpp
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

#include "al_utils/al_utils.h"

//ArffData*
//al::learning::initARFF (ArffParser*& arff_parser, std::string file_name)
//{
//  if (arff_parser == NULL)
//  {
//    arff_parser = new ArffParser (file_name);
//  }
//  std::string line;
//  std::ifstream in_file (file_name.c_str ());
//
//  if (in_file.is_open ())
//  {
//    std::cout << "opened the file " << file_name << std::endl;
//    return arff_parser->parse ();
//  }
//  else
//    return NULL;
//}
//
//void
//al::learning::saveARFF (ArffData* data, const std::string &directory_name)
//{
//  std::fstream out_file;
//  std::string file_name = directory_name + "/" + data->get_relation_name () + ".arff";
//  out_file.open (file_name.c_str (), std::fstream::out);
//  if (out_file.is_open ())
//    std::cout << "opened the file " << file_name << std::endl;
//  else
//    std::cout << "couldn't opened the file " << file_name << std::endl;
//
//  data->write_arff (out_file);
//  out_file.close ();
//}

std::vector<std::string>
al::speech::tokenizeSentence (std::string speech_text)
{
  using namespace std;
  vector<string> tokens;

  istringstream iss (speech_text);
  copy (istream_iterator < string > (iss), istream_iterator<string> (), back_inserter < vector<string> > (tokens));

  return tokens;
}

sensor_msgs::PointCloud
al::perception::toPCMsg (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
{
  sensor_msgs::PointCloud msg_cloud;
  msg_cloud.points.resize (cloud_data->points.size ());
  for (uint j = 0; j < cloud_data->points.size (); j++)
  {
    msg_cloud.points[j].x = cloud_data->points[j].x;
    msg_cloud.points[j].y = cloud_data->points[j].y;
    msg_cloud.points[j].z = cloud_data->points[j].z;
    msg_cloud.header = cloud_data->header;
  }
  return msg_cloud;
}

bool
al::perception::readPCD (std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud_data)
{
  ROS_INFO("Loading the file: %s", file_name.c_str ());
  pointcloud_data.reset (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name.c_str (), *pointcloud_data) == -1)
  {
    PCL_ERROR("Couldn't read file\n");
    return false;
  }
  else
  {
    ROS_INFO("Loaded %d data points from %s", pointcloud_data->width * pointcloud_data->height, file_name.c_str ());
//    pointcloud_data_->header.frame_id = "base_footprint";
    return true;
  }
}

bool
al::perception::writePCD (std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data)
{
  std::string file_name_;
  ROS_INFO("SAVING to the file %s", file_name.c_str ());

  pcl::io::savePCDFileASCII (file_name_.c_str (), *pointcloud_data);
  ROS_INFO("SAVED %d points to %s", (int )pointcloud_data->points.size (), file_name.c_str ());
  return true;
}

bool
al::perception::writePCD (std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_data)
{
  std::string file_name_;
  ROS_INFO("SAVING to the file %s", file_name.c_str ());

  pcl::io::savePCDFileASCII (file_name_.c_str (), *pointcloud_data);
  ROS_INFO("SAVED %d points to %s", (int )pointcloud_data->points.size (), file_name.c_str ());
  return true;
}

std::string
al::facilities::toString (const int int_)
{
  std::stringstream s;
  s << int_;
  return s.str ();
}

int
al::facilities::toInt (const std::string str_)
{
  std::stringstream s (str_);
  int int_;
  s >> int_;
  return int_;
}

std::string
al::facilities::shapeEnumToString (int8_t shape_enum)
{
  std::string str_shape;
  switch (shape_enum)
  {
    case al_msgs::Shape::CUP:
      str_shape = "CUP";
      break;
    case al_msgs::Shape::BOX:
      str_shape = "BOX";
      break;
    case al_msgs::Shape::CYLINDER:
      str_shape = "CYLINDER";
      break;
    case al_msgs::Shape::BALL:
      str_shape = "BALL";
      break;
    case al_msgs::Shape::SPHERE:
      str_shape = "SPHERE";
      break;
    case al_msgs::Shape::CUBE:
      str_shape = "CUBE";
      break;
    default:
      std::cout << "this is impossible, a bug!" << std::endl;
      break;
  }
  return str_shape;
}

std::string
al::facilities::behaviorEnumToString (int8_t behavior_enum)
{
  std::string str_behavior;
  switch (behavior_enum)
  {
    case al_msgs::Behavior::TUCK_ARMS:
      str_behavior = "TUCK_ARMS";
      break;
    case al_msgs::Behavior::REACH:
      str_behavior = "REACH";
      break;
    case al_msgs::Behavior::COVER:
      str_behavior = "COVER";
      break;
    case al_msgs::Behavior::CALL:
      str_behavior = "CALL";
      break;
    case al_msgs::Behavior::TAKE:
      str_behavior = "TAKE";
      break;
    case al_msgs::Behavior::GIVE:
      str_behavior = "GIVE";
      break;
    case al_msgs::Behavior::GRASP_TOP:
      str_behavior = "GRASP_TOP";
      break;
    case al_msgs::Behavior::GRASP_SIDE:
      str_behavior = "GRASP_SIDE";
      break;
    case al_msgs::Behavior::PUSH_RGT:
      str_behavior = "PUSH_RGT";
      break;
    case al_msgs::Behavior::PUSH_LFT:
      str_behavior = "PUSH_LFT";
      break;
    case al_msgs::Behavior::PUSH_FWD:
      str_behavior = "PUSH_FWD";
      break;
    case al_msgs::Behavior::PUSH_BWD:
      str_behavior = "PUSH_BWD";
      break;
    case al_msgs::Behavior::PUSH_TOP_RGT:
      str_behavior = "PUSH_TOP_RGT";
      break;
    case al_msgs::Behavior::PUSH_TOP_LFT:
      str_behavior = "PUSH_TOP_LFT";
      break;
    case al_msgs::Behavior::PUSH_TOP_FWD:
      str_behavior = "PUSH_TOP_FWD";
      break;
    case al_msgs::Behavior::PUSH_TOP_BWD:
      str_behavior = "PUSH_TOP_BWD";
      break;
    case al_msgs::Behavior::SAY_PASS_ME:
      str_behavior = "SAY_PASS_ME";
      break;
    default:
      std::cout << "this is impossible, a bug!" << std::endl;
      break;
  }
  return str_behavior;
}

std::string
al::facilities::effectEnumToString (int8_t effect_enum)
{
  std::string str_effect;
  switch (effect_enum)
  {
    case al_msgs::Effect::NO_CHANGE:
      str_effect = "NO_CHANGE";
      break;
    case al_msgs::Effect::REACHED:
      str_effect = "REACHED";
      break;
    case al_msgs::Effect::MOVED_RIGHT:
      str_effect = "MOVED_RIGHT";
      break;
    case al_msgs::Effect::MOVED_LEFT:
      str_effect = "MOVED_LEFT";
      break;
    case al_msgs::Effect::MOVED_FORWARD:
      str_effect = "MOVED_FORWARD";
      break;
    case al_msgs::Effect::MOVED_BACKWARD:
      str_effect = "MOVED_BACKWARD";
      break;
    case al_msgs::Effect::DISAPPEARED:
      str_effect = "DISAPPEARED";
      break;
    case al_msgs::Effect::VANISHED:
      str_effect = "VANISHED";
      break;
    case al_msgs::Effect::ROTATED_CW:
      str_effect = "ROTATED_CW";
      break;
    case al_msgs::Effect::ROTATED_CCW:
      str_effect = "ROTATED_CCW";
      break;
    case al_msgs::Effect::TOPPLED_RGT:
      str_effect = "TOPPLED_RGT";
      break;
    case al_msgs::Effect::TOPPLED_LFT:
      str_effect = "TOPPLED_LFT";
      break;
    case al_msgs::Effect::TOPPLED_FWD:
      str_effect = "TOPPLED_FWD";
      break;
    case al_msgs::Effect::TOPPLED_BWD:
      str_effect = "TOPPLED_BWD";
      break;
    case al_msgs::Effect::ROLLED_RIGHT:
      str_effect = "ROLLED_RIGHT";
      break;
    case al_msgs::Effect::ROLLED_LEFT:
      str_effect = "ROLLED_LEFT";
      break;
    case al_msgs::Effect::ROLLED_FORWARD:
      str_effect = "ROLLED_FORWARD";
      break;
    case al_msgs::Effect::ROLLED_BACKWARD:
      str_effect = "ROLLED_BACKWARD";
      break;
    case al_msgs::Effect::GRASPED:
      str_effect = "GRASPED";
      break;
    case al_msgs::Effect::ACQUIRED:
      str_effect = "ACQUIRED";
      break;
    case al_msgs::Effect::RELEASED:
      str_effect = "RELEASED";
      break;
    case al_msgs::Effect::TAKEN:
      str_effect = "TAKEN";
      break;
    case al_msgs::Effect::GIVEN:
      str_effect = "GIVEN";
      break;
    case al_msgs::Effect::GOT_ATTENTION:
      str_effect = "GOT_ATTENTION";
      break;
    case al_msgs::Effect::TOPPLED:
      str_effect = "TOPPLED";
      break;
    case al_msgs::Effect::GOT_SEATED:
      str_effect = "GOT_SEATED";
      break;
    default:
      std::cout << "this is impossible, a bug!" << std::endl;
      break;
  }
  return str_effect;
}

std::string
al::facilities::specEnumToString (int8_t spec_enum)
{
  std::string str_spec = "";
  switch (spec_enum)
  {
    case al_msgs::Spec::THIN:
      str_spec = "THIN";
      break;
    case al_msgs::Spec::THICK:
      str_spec = "THICK";
      break;
    case al_msgs::Spec::ROUND:
      str_spec = "ROUND";
      break;
    case al_msgs::Spec::EDGY:
      str_spec = "EDGY";
      break;
    case al_msgs::Spec::TALL:
      str_spec = "TALL";
      break;
    case al_msgs::Spec::SHORT:
      str_spec = "SHORT";
      break;
    case al_msgs::Spec::BIG:
      str_spec = "BIG";
      break;
    case al_msgs::Spec::SMALL:
      str_spec = "SMALL";
      break;
    default:
      std::cout << "Adjective conversion failed, check al_utils.cpp!" << std::endl;
      break;
  }
  return str_spec;
}

int8_t
al::facilities::effectStringToEnum (std::string effect_str)
{
  for (uint i = 0; i <= al_msgs::Effect::MAX_EFFECT_INDEX; i++)
  {
    if (effect_str == effectEnumToString (i))
      return i;
  }
  return -1;
}

std::string
al::facilities::adjTypeEnumToString (int adj_type_enum)
{
  std::string str_adj_type;
  switch (adj_type_enum)
  {
    case al::learning::adj_thin_thick:
      str_adj_type = "THIN_THICK";
      break;
    case al::learning::adj_round_edgy:
      str_adj_type = "ROUND_EDGY";
      break;
    case al::learning::adj_tall_short:
      str_adj_type = "TALL_SHORT";
      break;
    case al::learning::adj_big_small:
      str_adj_type = "BIG_SMALL";
      break;
    default:
      ROS_WARN("there is no such adjective type, this is probably a bug!");
      break;
  }
  return str_adj_type;
}

//std::vector<float>
//al::facilities::cvtRawFeatureVector (const al_msgs::FeatureVector& feature_vector)
//{
//  std::vector<float> raw_feature_vector;
//
//  for (uint i = 0; i < feature_vector.features.size (); i++)
//  {
//    if (feature_vector.features[i].val_type == al_msgs::Feature::SINGLE_VALUED)
//    {
//      raw_feature_vector.push_back (feature_vector.features[i].avg);
//      continue;
//    }
//    else
//    {
//      raw_feature_vector.push_back (feature_vector.features[i].min);
//      raw_feature_vector.push_back (feature_vector.features[i].max);
//      raw_feature_vector.push_back (feature_vector.features[i].avg);
//      raw_feature_vector.push_back (feature_vector.features[i].dev);
//      raw_feature_vector.push_back (feature_vector.features[i].var);
//
//      for (uint j = 0; j < feature_vector.features[i].his.size (); j++)
//      {
//        raw_feature_vector.push_back (feature_vector.features[i].his[j]);
//      }
//    }
//  }
//  return raw_feature_vector;
//}

std::vector<double>
al::facilities::cvtRawFeatureVector (const al_msgs::FeatureVector& feature_vector)
{
  std::vector<double> raw_feature_vector;

  for (uint i = 0; i < feature_vector.features.size (); i++)
  {
    if (feature_vector.features[i].val_type == al_msgs::Feature::SINGLE_VALUED)
    {
      raw_feature_vector.push_back (feature_vector.features[i].avg);
      continue;
    }
    else
    {
      raw_feature_vector.push_back (feature_vector.features[i].min);
      raw_feature_vector.push_back (feature_vector.features[i].max);
      raw_feature_vector.push_back (feature_vector.features[i].avg);
      raw_feature_vector.push_back (feature_vector.features[i].dev);
      raw_feature_vector.push_back (feature_vector.features[i].var);

      for (uint j = 0; j < feature_vector.features[i].his.size (); j++)
      {
        raw_feature_vector.push_back (feature_vector.features[i].his[j]);
      }
    }
  }
  return raw_feature_vector;
}

bool
al::facilities::createFeatureVector (const std::vector<float>& raw_data, al_msgs::FeatureVector& feature_vector)
{
  //assumes a feature vector with the structure of:
  // <<azi_normal>> <<zen_normal>> <<curvs_min>> <<curvs_max>> <<shape_indices>>
  // <x> <y> <z> <t> <dim_x> <dim_y> <dim_z> <presence>
  // <t_x> <t_y> <t_z> <t_roll> <t_pitch> <t_yaw> <h_roll> <h_pitch> <h_yaw> <human_presence>

  int f_vector_size = 2 * (5 + perception::N_NORMAL_HISTOGRAM_BINS) + 2 * (5 + perception::N_CURVATURE_HISTOGRAM_BINS)
      + (5 + perception::N_SHAPE_ID_HISTOGRAM_BINS) + 8 + 10;

  if ((int)raw_data.size () != f_vector_size)
    return false;
  else
  {
    feature_vector.features.resize (23);

    uint f_id = 0;
    uint f_index = 0;
    al_msgs::Feature f_azi_normal;
    f_azi_normal.type = al_msgs::Feature::NORMAL_AZI;
    f_azi_normal.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
    f_azi_normal.min = raw_data[f_index + 0];
    f_azi_normal.max = raw_data[f_index + 1];
    f_azi_normal.avg = raw_data[f_index + 2];
    f_azi_normal.dev = raw_data[f_index + 3];
    f_azi_normal.var = raw_data[f_index + 4];
    f_index += 5;
    f_azi_normal.his.resize (perception::N_NORMAL_HISTOGRAM_BINS);

    for (uint i = 0; i < (uint)perception::N_NORMAL_HISTOGRAM_BINS; i++)
      f_azi_normal.his[i] = raw_data[f_index + i];
    feature_vector.features[f_id++] = f_azi_normal;
    f_index += perception::N_NORMAL_HISTOGRAM_BINS;

    al_msgs::Feature f_zen_normal;
    f_zen_normal.type = al_msgs::Feature::NORMAL_ZEN;
    f_zen_normal.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
    f_zen_normal.min = raw_data[f_index + 0];
    f_zen_normal.max = raw_data[f_index + 1];
    f_zen_normal.avg = raw_data[f_index + 2];
    f_zen_normal.dev = raw_data[f_index + 3];
    f_zen_normal.var = raw_data[f_index + 4];
    f_index += 5;
    f_zen_normal.his.resize (perception::N_NORMAL_HISTOGRAM_BINS);

    for (uint i = 0; i < (uint)perception::N_NORMAL_HISTOGRAM_BINS; i++)
      f_zen_normal.his[i] = raw_data[f_index + i];
    feature_vector.features[f_id++] = f_zen_normal;
    f_index += perception::N_NORMAL_HISTOGRAM_BINS;

    al_msgs::Feature f_curvs_min;
    f_curvs_min.type = al_msgs::Feature::CURV_MIN;
    f_curvs_min.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
    f_curvs_min.min = raw_data[f_index + 0];
    f_curvs_min.max = raw_data[f_index + 1];
    f_curvs_min.avg = raw_data[f_index + 2];
    f_curvs_min.dev = raw_data[f_index + 3];
    f_curvs_min.var = raw_data[f_index + 4];
    f_index += 5;
    f_curvs_min.his.resize (perception::N_CURVATURE_HISTOGRAM_BINS);

    for (uint i = 0; i < (uint)perception::N_CURVATURE_HISTOGRAM_BINS; i++)
      f_curvs_min.his[i] = raw_data[f_index + i];
    feature_vector.features[f_id++] = f_curvs_min;
    f_index += perception::N_CURVATURE_HISTOGRAM_BINS;

    al_msgs::Feature f_curvs_max;
    f_curvs_max.type = al_msgs::Feature::CURV_MAX;
    f_curvs_max.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
    f_curvs_max.min = raw_data[f_index + 0];
    f_curvs_max.max = raw_data[f_index + 1];
    f_curvs_max.avg = raw_data[f_index + 2];
    f_curvs_max.dev = raw_data[f_index + 3];
    f_curvs_max.var = raw_data[f_index + 4];
    f_index += 5;
    f_curvs_max.his.resize (perception::N_CURVATURE_HISTOGRAM_BINS);

    for (uint i = 0; i < (uint)perception::N_CURVATURE_HISTOGRAM_BINS; i++)
      f_curvs_max.his[i] = raw_data[f_index + i];
    feature_vector.features[f_id++] = f_curvs_max;
    f_index += perception::N_CURVATURE_HISTOGRAM_BINS;

    al_msgs::Feature f_shape_indices;
    f_shape_indices.type = al_msgs::Feature::SHAPE_INDEX;
    f_shape_indices.val_type = al_msgs::Feature::DISPERSIVE_VALUED;
    f_shape_indices.min = raw_data[f_index + 0];
    f_shape_indices.max = raw_data[f_index + 1];
    f_shape_indices.avg = raw_data[f_index + 2];
    f_shape_indices.dev = raw_data[f_index + 3];
    f_shape_indices.var = raw_data[f_index + 4];
    f_index += 5;
    f_shape_indices.his.resize (perception::N_SHAPE_ID_HISTOGRAM_BINS);

    for (uint i = 0; i < (uint)perception::N_SHAPE_ID_HISTOGRAM_BINS; i++)
      f_shape_indices.his[i] = raw_data[f_index + i];
    feature_vector.features[f_id++] = f_shape_indices;
    f_index += perception::N_SHAPE_ID_HISTOGRAM_BINS;

    for (; f_index < raw_data.size (); f_index++)
    {
      al_msgs::Feature f;
      //    f_x.type = al_msgs::Feature::POS_X;
      f.val_type = al_msgs::Feature::SINGLE_VALUED;
      f.avg = raw_data[f_index];
      feature_vector.features[f_id++] = f;
    }
    f_index = 5;
    feature_vector.features[f_id++].type = al_msgs::Feature::POS_X;
    feature_vector.features[f_id++].type = al_msgs::Feature::POS_Y;
    feature_vector.features[f_id++].type = al_msgs::Feature::POS_Z;
    feature_vector.features[f_id++].type = al_msgs::Feature::ROT_T;
    feature_vector.features[f_id++].type = al_msgs::Feature::DIM_X;
    feature_vector.features[f_id++].type = al_msgs::Feature::DIM_Y;
    feature_vector.features[f_id++].type = al_msgs::Feature::DIM_Z;
    feature_vector.features[f_id++].type = al_msgs::Feature::OBJECT_PRESENCE;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_TORSO_X;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_TORSO_Y;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_TORSO_Z;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_TORSO_ROLL;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_TORSO_PITCH;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_TORSO_YAW;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_HEAD_ROLL;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_HEAD_PITCH;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_HEAD_YAW;
    feature_vector.features[f_id++].type = al_msgs::Feature::HUMAN_PRESENCE;
  }
  return true;
}

arm_navigation_msgs::CollisionObject*
al::memory::getCollisionObject (std::vector<arm_navigation_msgs::CollisionObject> &collision_objects, std::string id)
{
  arm_navigation_msgs::CollisionObject* collision_object = NULL;
  for (uint i = 0; i < collision_objects.size (); i++)
  {
    if (collision_objects[i].id == id)
    {
      collision_object = &collision_objects[i];
    }
  }
  return collision_object;
}

bool
al::behavior::getTransformBetween (const std::string parent_frame, const std::string child_frame,
                                   tf::StampedTransform& transform)
{
  static tf::TransformListener listener;

  bool transformation_healthy = false;
  int n_trial = 3;
  while (!transformation_healthy && n_trial > 0)
  {
    n_trial--;
    try
    {
      listener.lookupTransform (parent_frame.c_str (), child_frame.c_str (), ros::Time (0), transform);
      ros::spinOnce ();
      transformation_healthy = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what ());
      transformation_healthy = false;
    }
  }
  return transformation_healthy;
}

std_msgs::ColorRGBA
al::viz::cvtHSVToRGB (float hue, float saturation, float value)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;

  float hue_prime = hue / 60;
  float chrome = value * saturation;
  float x = chrome * (1 - fabs (fmod (hue_prime, 2) - 1));
  float r_prime, g_prime, b_prime;

  if (hue_prime >= 0 && hue_prime < 1)
  {
    r_prime = chrome;
    g_prime = x;
    b_prime = 0;
  }
  else if (hue_prime >= 1 && hue_prime < 2)
  {
    r_prime = x;
    g_prime = chrome;
    b_prime = 0;
  }
  else if (hue_prime >= 2 && hue_prime < 3)
  {
    r_prime = 0;
    g_prime = chrome;
    b_prime = x;
  }
  else if (hue_prime >= 3 && hue_prime < 4)
  {
    r_prime = 0;
    g_prime = x;
    b_prime = chrome;
  }
  else if (hue_prime >= 4 && hue_prime < 5)
  {
    r_prime = x;
    g_prime = 0;
    b_prime = chrome;
  }
  else if (hue_prime >= 5 && hue_prime < 6)
  {
    r_prime = chrome;
    g_prime = 0;
    b_prime = x;
  }
  else
  {
    r_prime = 0;
    g_prime = 0;
    b_prime = 0;
  }
  float m = value - chrome;
  color.r = r_prime + m;
  color.g = g_prime + m;
  color.b = b_prime + m;

  return color;
}

std_msgs::ColorRGBA
al::viz::colorize (int color_id, float alpha)
{
  std_msgs::ColorRGBA color;
  color.a = alpha;
  switch (color_id)
  {
    case WHITE:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      break;
    case RED:
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      break;
    case GREEN:
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      break;
    case BLUE:
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      break;
    case YELLOW:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.0;
      break;
    case CYAN:
      color.r = 0.0;
      color.g = 1.0;
      color.b = 1.0;
      break;
    case MAGENTA:
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      break;
    case BLACK:
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.0;
      break;
    case GREY:
      color.r = 0.4;
      color.g = 0.4;
      color.b = 0.4;
      color.a = 1.0;
      break;
    default:
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }
  return color;
}

//TODO: make this more generic
//src is CV_32FC1 type image and dest is CV_8UC1
bool
al::viz::cvtTo8BitImage (const cv::Mat &src, cv::Mat &dest, float min_val, float max_val, bool discard_zeroes)
{
  dest = cv::Mat::zeros (src.rows, src.cols, CV_8UC1);
  for (int ri = 0; ri < src.rows; ri++)
    for (int ci = 0; ci < src.cols; ci++)
      if (src.at<float> (ri, ci) || !discard_zeroes)
        dest.at<char> (ri, ci) = (char)(255 * (src.at<float> (ri, ci) - min_val) / (max_val - min_val));

  return true;
}

void
al::viz::vizPyramid (const std::vector<cv::Mat> &img_levels)
{
  float min_range = FLT_MAX;
  float max_range = FLT_MIN;

  for (uint l = 0; l < img_levels.size (); l++)
  {
    for (int ri = 0; ri < img_levels[l].rows; ri++)
    {
      for (int ci = 0; ci < img_levels[l].cols; ci++)
      {
//        if (img_levels[l].at<float> (ri, ci) > 0.01) //if non-zero
        if (img_levels[l].at<float> (ri, ci) > 0.80) //if non-zero
        {
          if (img_levels[l].at<float> (ri, ci) < min_range)
            min_range = img_levels[l].at<float> (ri, ci);

          if (img_levels[l].at<float> (ri, ci) > max_range)
            max_range = img_levels[l].at<float> (ri, ci);
        }
      }
    }
  }

  for (uint l = 0; l < img_levels.size (); l++)
  {
    cv::Mat img_8;
    al::viz::cvtTo8BitImage (img_levels[l], img_8, min_range, max_range);
    std::string window_name = "Display window_" + al::facilities::toString ((int)l) + "_"
        + al::facilities::toString ((int)l);
    cv::namedWindow (window_name.c_str (), CV_WINDOW_AUTOSIZE); // Create a window for display.
    cv::imshow (window_name.c_str (), img_8); // Show our image inside it.
  }
}

std::string
al::system::execCmd (std::string sys_cmd)
{
  FILE* pipe = popen (sys_cmd.c_str (), "r");
  if (!pipe)
    return "ERROR";
  char buffer[512];
  std::string result = "";
  while (!feof (pipe))
  {
    if (fgets (buffer, 512, pipe) != NULL)
      result += buffer;
  }
  pclose (pipe);
  return result;
}

int
al::sim::spawnObjectByModel (std::string model_name)
{
  std::string model_file_path = system::execCmd ("rospack find sim_environment");
  if (!model_file_path.size ())
  {
    ROS_ERROR("Couldn't file the package sim_environment");
    return -1;
  }

  model_file_path.erase (model_file_path.find ("\n"));
  model_file_path.append ("/objects/" + model_name + ".model");
  std::cout << model_file_path << std::endl;
  std::string model_description = model_name + "_description";

  if (system::execSysCmd (("rosparam load " + model_file_path + " " + model_description).c_str ()))
    return -1;

  int out =
      system::execSysCmd (
          ("rosrun gazebo spawn_model -param " + model_description + " -gazebo -model " + model_name + " -x 1 -z 0.75").c_str ());
  if (out)
    return -1;
  else
    return 1;
}

int
al::system::execSysCmd (std::string sys_cmd)
{
  return std::system (sys_cmd.c_str ());
}

int
al::system::getProcessIdByName (std::string p_name)
{
  int pid = 0;
  //if tabletop_segmentation service is already running, don't re-roslaunch
  std::string str_pid = execCmd ("pidof " + p_name);
  std::stringstream ss (str_pid);
  ss >> pid;

  return pid;
}

int
al::system::killProcessByName (std::string p_name, int kill_signal)
{
  return kill (getProcessIdByName (p_name), kill_signal);
}

int
al::system::killNodeByName (std::string n_name)
{
  return execSysCmd (("rosnode kill " + n_name).c_str ());
}

void
al::system::changeInputMode (int dir)
{
  static struct termios oldt, newt;
  if (dir == 1)
  {
    tcgetattr (STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr (STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr (STDIN_FILENO, TCSANOW, &oldt);
}

char
al::system::keyboardHit ()
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET(STDIN_FILENO, &rdfs);

  select (STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
  return (char)FD_ISSET (STDIN_FILENO, &rdfs);
}

bool
al::system::getKey (char& key)
{
  if (keyboardHit ())
  {
    key = getchar ();
    return true;
  }
  else
    return false;
}

bool
al::system::getPackagePath (std::string &package_path, const std::string ros_package_name)
{
  package_path = ros::package::getPath (ros_package_name);
  if (!package_path.length ())
    return false;
  else
    return true;
}

bool
al::system::getFileList (std::vector<std::string> &file_list, const std::string parent_dir_name)
{
  DIR *dir;
  struct dirent *ent;
  dir = opendir (parent_dir_name.c_str ());
  if (dir != NULL)
  {
    while ((ent = readdir (dir)) != NULL)
    {
      std::string file_name (ent->d_name);
      //      if (file_name != "." || file_name != "..")
      if (file_name.find (".arff") != std::string::npos)
      {
        std::cout << file_name << std::endl;
        file_list.push_back (file_name);
      }
    }
    closedir (dir);
    return true;
  }
  else
  {
    perror ("");
    return false;
  }
}

float
al::math::fPrecision (const ros::Time &time, const int n_int_digits, const int n_floating_digits)
{
  float timestamp = (float)((int)time.sec % (int)exp10 (n_int_digits)
      + round ((int)time.nsec * exp10 (-9 + n_floating_digits)) / exp10 (n_floating_digits));

  return timestamp;
}

float
al::math::fPrecision (const float number, const int n_int_digits, const int n_floating_digits)
{
  double int_digits, floating_digits;
  floating_digits = modf (number, &int_digits);

  return (int)int_digits % (int)exp10 (n_int_digits)
      + round (floating_digits * exp10 (n_floating_digits)) / exp10 (n_floating_digits);
}

float
al::math::fPrecision (const ros::Duration &duration, const int n_floating_digits)
{
  float timestamp = (float)(duration.sec
      + round ((int)duration.nsec * exp10 (-9 + n_floating_digits)) / exp10 (n_floating_digits));

  return timestamp;
}

al::math::Polygon::Polygon ()
{
  area_ = -1;
}

bool
al::math::Polygon::setVertices (const std::vector<tf::Vector3> vertices)
{
  if (vertices.size () < 3)
  {
    ROS_WARN("this is not a polygon!");
    return false;
  }

  vertices_ = vertices;

  //  std::cout << "before\n";
  //  for (uint i = 0; i < 4; i++)
  //    std::cout << vertices_[i].x () << " " << vertices_[i].y () << " " << vertices_[i].z () << "\n";
  //sort vertices in the ccw direction, picks the right most one as the first vertex

  //get the right most vertex
  uint right_most_vertex_index = 0;
  double max_x = vertices_[0].x ();
  for (uint i = 1; i < vertices_.size (); i++)
  {
    if (vertices_[i].x () > max_x)
    {
      right_most_vertex_index = i;
      max_x = vertices_[i].x ();
    }
  }
  tf::Vector3* v_right_most = &vertices_[right_most_vertex_index];

  //sort other vertices according to their angles w.r.t. v_right_most
  std::map<float, tf::Vector3> map_vertices;
  for (uint i = 0; i < vertices_.size (); i++)
  {
    if (i == right_most_vertex_index)
      continue;

    float angle = getAngle (tf::Vector3 (vertices_[i] - *v_right_most));
    map_vertices[angle] = vertices_[i];
  }

  std::map<float, tf::Vector3>::iterator it;
  vertices_[0] = vertices_[right_most_vertex_index];
  uint index = 1;
  for (it = map_vertices.begin (); it != map_vertices.end (); it++)
  {
    //    std::cout << (float)it->first << std::endl;
    vertices_[index] = it->second;
    index++;
  }
  //  std::cout << "after\n";
  //  for (uint i = 0; i < 4; i++)
  //    std::cout << vertices_[i].x () << " " << vertices_[i].y () << " " << vertices_[i].z () << "\n";

  return true;
}

al::math::Polygon::Polygon (const std::vector<tf::Vector3> vertices)
{
  area_ = -1;
  setVertices (vertices);
}

//calculates the area of a convex polygon by triangulation
float
al::math::Polygon::getArea ()
{
  if (vertices_.size () < 3)
    return 0;

  if (area_ != -1)
    return area_;
  else
  {
    area_ = 0;
    for (uint i = 1; i < vertices_.size () - 1; i++)
    {
      tf::Vector3 v1 = vertices_[i] - vertices_[0];
      tf::Vector3 v2 = vertices_[(i + 1)] - vertices_[0];
      area_ += getAreaBetween (v1, v2);
    }
    return area_;
  }
}

float
al::math::getAngle (tf::Vector3 v)
{
  float angle = atan2 (v.y (), v.x ());
  if (angle < 0)
    angle += 2 * PI;

  return angle;
}

float
al::math::get2DCross (tf::Vector3 v1, tf::Vector3 v2)
{
  return v1.x () * v2.y () - v2.x () * v1.y ();
}

float
al::math::fRand (float fMin, float fMax)
{
  float f = (float)rand () / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

int
al::math::iRand (int min, int max)
{
  int range = (max - min + 1);
  float rand_weight = rand () / (RAND_MAX + 1.0);
  //  int rand_num = min + (int)(range * rand_weight);
  return min + (int)(range * rand_weight);
}

float
al::math::getAngleBetween (tf::Vector3 v1, tf::Vector3 v2)
{
  if (fabs (v1.length ()) < 0.001 || fabs (v2.length ()) < 0.001)
  {
    ROS_WARN("division by zero!");
    return 0;
  }

  return acos (v1.dot (v2) / (v1.length () * v2.length ()));
}

float
al::math::getAreaBetween (tf::Vector3 v1, tf::Vector3 v2)
{
  return v1.length () * v2.length () * sin (getAngleBetween (v1, v2)) / 2.0;
}

float
al::math::getIntersectionVolume (const arm_navigation_msgs::CollisionObject &obj1,
                                 const arm_navigation_msgs::CollisionObject &obj2)
{
  //this is guaranteed to be a convex polygon
  Polygon rect1 = getXYCrossSection (obj1);
  Polygon rect2 = getXYCrossSection (obj2);
  //  for (uint i = 0; i < rect1.vertices_.size (); i++)
  //    std::cout << rect1.vertices_[i].x () << " " << rect1.vertices_[i].y () << " " << rect1.vertices_[i].z () << "\n";
  //  std::cout << "\n";
  //  for (uint i = 0; i < rect2.vertices_.size (); i++)
  //    std::cout << rect2.vertices_[i].x () << " " << rect2.vertices_[i].y () << " " << rect2.vertices_[i].z () << "\n";

  /* test
   //  std::cout << "obj 1\n";
   //  for (uint i = 0; i < rect1.vertices_.size (); i++)
   //    std::cout << rect1.vertices_[i].x () << " " << rect1.vertices_[i].y () << " " << rect1.vertices_[i].z ()
   //        << std::endl;
   //  std::cout << "obj 2\n";
   //
   //  for (uint i = 0; i < rect2.vertices_.size (); i++)
   //    std::cout << rect2.vertices_[i].x () << " " << rect2.vertices_[i].y () << " " << rect2.vertices_[i].z ()
   //        << std::endl;

   //test: modify rect2
   //  rect2.vertices_ = rect1.vertices_;
   //  tf::Vector3 shift;
   //shift in diagonal
   //  shift.setValue (0.02, -0.02, 0.50);

   //shift in one direction
   //  shift.setValue (0.02, 0.0, 0.50);

   //no shift, worst case
   //  shift.setValue (0, 0, 0);

   for (uint i = 0; i < rect2.vertices_.size (); i++)
   rect2.vertices_[i] += shift;
   */
  std::vector<tf::Vector3> inner_polygon_pts = getInnerPoints (rect1, rect2);
  std::vector<tf::Vector3> intersection_polygon_pts = getIntersectionPoints (rect1, rect2);

  intersection_polygon_pts.insert (intersection_polygon_pts.end (), inner_polygon_pts.begin (),
                                   inner_polygon_pts.end ());

  if (!intersection_polygon_pts.size ())
    return 0;

  for (uint i = 0; i < intersection_polygon_pts.size (); i++)
  {
    for (uint j = 0; j < intersection_polygon_pts.size (); j++)
    {
      //delete too close points, they do not contribute to the area
      if ((intersection_polygon_pts[i] - intersection_polygon_pts[j]).length () < 0.001 && i != j)
      {
        intersection_polygon_pts.erase (intersection_polygon_pts.begin () + j);
        --j;
      }
    }
  }

  if (intersection_polygon_pts.size () < 3)
    ROS_WARN("there is something wrong with the intersection detection!");

  //  for (uint i = 0; i < intersection_polygon_pts.size (); i++)
  //    std::cout << intersection_polygon_pts[i].x () << " " << intersection_polygon_pts[i].y () << " "
  //        << intersection_polygon_pts[i].z () << "\n";

  Polygon intersection_polygon (intersection_polygon_pts);
  float intersection_area = intersection_polygon.getArea ();
  //  std::cout << "intersection_area: " << intersection_area << std::endl;
  //  std::cout << "area:              " << obj1.shapes[0].dimensions[0] * obj1.shapes[0].dimensions[1] << "\n";

  //if they intersect in the z direction
  std::map<float, int> map_z_vals_vs_box_id; //id 1 for obj1, 2 for obj2
  map_z_vals_vs_box_id[obj1.poses[0].position.z - obj1.shapes[0].dimensions[2] / 2] = 1;
  map_z_vals_vs_box_id[obj1.poses[0].position.z + obj1.shapes[0].dimensions[2] / 2] = 1;
  map_z_vals_vs_box_id[obj2.poses[0].position.z - obj2.shapes[0].dimensions[2] / 2] = 2;
  map_z_vals_vs_box_id[obj2.poses[0].position.z + obj2.shapes[0].dimensions[2] / 2] = 2;

  //if first two or second two of the map entries belong to the same box, they do not intersect
  std::map<float, int>::iterator it = map_z_vals_vs_box_id.begin ();
  if (it->second == (++it)->second) //intersecting volume is zero
    return 0;
  //if second and third entries belong to the same box, then the intersection height is its height
  else
  {
    //second and third belong to different boxes
    //or same box that is included by other.
    //intersection height is this difference in anyway.
    it = ++map_z_vals_vs_box_id.begin (); //just to make sure

    float delta_z = fabs (it->first - (++it)->first);

    //    std::cout << "intersection_vol: " << intersection_area * delta_z << std::endl;
    //    std::cout << "vol1:             " << obj1.shapes[0].dimensions[0] * obj1.shapes[0].dimensions[1]
    //        * obj1.shapes[0].dimensions[2] << "\n";
    //    std::cout << "vol2:             " << obj2.shapes[0].dimensions[0] * obj2.shapes[0].dimensions[1]
    //        * obj2.shapes[0].dimensions[2] << "\n";

    return delta_z * intersection_area;
  }
}

float
al::math::getVolume (const arm_navigation_msgs::CollisionObject &obj)
{
  if (obj.shapes.size () > 0)
  {
    if (obj.shapes[0].dimensions.size () == 3)
      return obj.shapes[0].dimensions[0] * obj.shapes[0].dimensions[1] * obj.shapes[0].dimensions[2];
    else
      //this is not a box volume
      return 0;
  }
  else
    return 0;
}

bool
al::math::innerPoint (const tf::Vector3 &v, const Polygon& rect)
{
  for (uint i = 0; i < rect.vertices_.size (); i++)
  {
    tf::Vector3 v1 = rect.vertices_[(i + 1) % rect.vertices_.size ()] - rect.vertices_[i];
    v1.setZ (0);
    tf::Vector3 v2 = v - rect.vertices_[i];
    v2.setZ (0);

    // if cross product is >= 0, the point is inside the rect
    //this will cover colinearity cases while intersection cannot detect these cases
    if (v1.cross (v2).z () < 0)
      return false;
  }
  //  std::cout << " inner   : " << v.x () << " " << v.y () << " " << v.z () << std::endl;
  return true;
}

std::vector<tf::Vector3>
al::math::getInnerPoints (const Polygon& rect1, const Polygon& rect2)
{
  std::vector<tf::Vector3> inner_points;
  /*
   tf::Vector3 v;

   //test 1: an inside point
   //  v.setValue (0.60, -0.07, 0.67);

   //test 2: randomly generated inside points
   //  v.setX (fRand (rect1.vertices_[1].x (), rect1.vertices_[0].x ()));
   //  v.setY (fRand (rect1.vertices_[2].y (), rect1.vertices_[0].y ()));
   //  v.setZ (fRand (-0.1, 0.1));

   //test 3: boundary -corner
   //  int i = iRand (0, 3);
   //  std::cout << "corner index:" << i << std::endl;
   //  v = rect1.vertices_[i];

   //test 4: boundary
   //  int i = iRand (0, 3);
   //  float w = fRand (0, 1);
   //  v = rect1.vertices_[i] * w + (rect1.vertices_[(i + 1) % rect1.vertices_.size ()] * (1 - w));
   //  std::cout << "corner index:" << i << " weight:" << w << std::endl;

   //test 5: randomly generated outside points
   //  v.setX (fRand (rect1.vertices_[1].x (), rect1.vertices_[0].x ()));
   //  v.setY (fRand (rect1.vertices_[2].y () - 0.01, -1.0));
   //  v.setZ (fRand (-1.0, 1.0));

   if (innerPoint (v, rect1))
   {
   std::cout << " INNER" << std::endl;
   return inner_points;
   }
   */

  for (uint i = 0; i < rect1.vertices_.size (); i++)
  {
    if (innerPoint (rect1.vertices_[i], rect2))
      inner_points.push_back (rect1.vertices_[i]);
  }

  for (uint i = 0; i < rect2.vertices_.size (); i++)
  {
    if (innerPoint (rect2.vertices_[i], rect1))
      inner_points.push_back (rect2.vertices_[i]);
  }

  return inner_points;
}

bool
al::math::intersectionPoint (const LineSegment seg1, const LineSegment seg2, tf::Vector3 &v)
{
  tf::Vector3 vec1 = seg1.end_ - seg1.start_; //r
  tf::Vector3 vec2 = seg2.end_ - seg2.start_; //s

  float vec1_x_vec2 = get2DCross (vec1, vec2);
  float inter_seg_x_vec1 = get2DCross ((seg2.start_ - seg1.start_), vec1);
  float inter_seg_x_vec2 = get2DCross ((seg2.start_ - seg1.start_), vec2);

  if (fabs (vec1_x_vec2) < 0.0001) //two line segments are parallel
  {
    //assuming line segments are never a point (i.e. start!=end)
    if (fabs (inter_seg_x_vec1) < 0.0001)
    {
      ROS_DEBUG("segments are parallel and colinear, a segment intersection.");
      //This can be specially handled
      return false;
    }
    else
    {
      ROS_DEBUG("segments are parallel, never intersect");
      return false;
    }
  }
  else
  {
    float int_dist_on_seg_2 = inter_seg_x_vec1 / vec1_x_vec2;
    float int_dist_on_seg_1 = inter_seg_x_vec2 / vec1_x_vec2;
    //    std::cout << "t: " << int_dist_on_seg_1 << " " << "u: " << int_dist_on_seg_2 << "\n";

    //    std::cout << "seg1: " << seg1.start_.x () << " " << seg1.start_.y () << " " << seg1.start_.z () << "\t"
    //        << seg1.end_.x () << " " << seg1.end_.y () << " " << seg1.end_.z () << "\n";
    //    std::cout << "seg2: " << seg2.start_.x () << " " << seg2.start_.y () << " " << seg2.start_.z () << "\t"
    //        << seg2.end_.x () << " " << seg2.end_.y () << " " << seg2.end_.z () << "\n";

    if (int_dist_on_seg_1 >= 0.0 && int_dist_on_seg_1 <= 1.0 && int_dist_on_seg_2 >= 0.0 && int_dist_on_seg_2 <= 1.0)
    {
      ROS_DEBUG("intersection detected");

      v.setX (seg2.start_.x () + int_dist_on_seg_2 * vec2.x ());
      v.setY (seg2.start_.y () + int_dist_on_seg_2 * vec2.y ());
      v.setZ (seg2.start_.z () + int_dist_on_seg_2 * vec2.z ());
      //      std::cout << " intersect: " << v.x () << " " << v.y () << " " << v.z () << "\n";
      return true;
    }
    else
    {
      ROS_DEBUG("extended intersection detected");

      v.setX (seg2.start_.x () + int_dist_on_seg_2 * vec2.x ());
      v.setY (seg2.start_.y () + int_dist_on_seg_2 * vec2.y ());
      v.setZ (seg2.start_.z () + int_dist_on_seg_2 * vec2.z ());
      //      std::cout << "e_intersect:" << v.x () << " " << v.y () << " " << v.z () << "\n";
      return false;
    }
  }
}

std::vector<tf::Vector3>
al::math::getIntersectionPoints (const Polygon& rect1, const Polygon& rect2)
{
  std::vector<tf::Vector3> intersection_pts;
  LineSegment seg1;
  LineSegment seg2;

  for (uint i = 0; i < rect1.vertices_.size (); i++)
  {
    seg1.start_ = rect1.vertices_[i];
    seg1.start_.setZ (0);
    seg1.end_ = rect1.vertices_[(i + 1) % rect1.vertices_.size ()];
    seg1.end_.setZ (0);
    for (uint j = 0; j < rect2.vertices_.size (); j++)
    {
      seg2.start_ = rect2.vertices_[j];
      seg2.start_.setZ (0);
      seg2.end_ = rect2.vertices_[(j + 1) % rect2.vertices_.size ()];
      seg2.end_.setZ (0);
      tf::Vector3 intersection_pt;
      if (intersectionPoint (seg1, seg2, intersection_pt))
        intersection_pts.push_back (intersection_pt);
    }
  }
  return intersection_pts;
}

//assumes box shaped object, one of the main assumptions actually.
al::math::Polygon
al::math::getXYCrossSection (const arm_navigation_msgs::CollisionObject &obj)
{
  std::vector<tf::Vector3> vertices (4);
  geometry_msgs::Point pos_obj = obj.poses[0].position;
  tf::Vector3 pos (pos_obj.x, pos_obj.y, 0.0);
  geometry_msgs::Quaternion ori_obj = obj.poses[0].orientation;
  tf::Quaternion q;
  tf::quaternionMsgToTF (ori_obj, q);

  double yaw_angle = tf::getYaw (q);
  geometry_msgs::Vector3 dim_obj;
  dim_obj.x = obj.shapes[0].dimensions[0];
  dim_obj.y = obj.shapes[0].dimensions[1];
  dim_obj.z = obj.shapes[0].dimensions[2];

  //not rotated, not translated
  vertices[0].setValue (dim_obj.x / 2, dim_obj.y / 2, 0.0);
  vertices[1].setValue (-dim_obj.x / 2, dim_obj.y / 2, 0.0);
  vertices[2].setValue (-dim_obj.x / 2, -dim_obj.y / 2, 0.0);
  vertices[3].setValue (dim_obj.x / 2, -dim_obj.y / 2, 0.0);

  for (uint i = 0; i < 4; i++)
  {
    //rotate
    vertices[i] = vertices[i].rotate (tf::Vector3 (0, 0, 1), yaw_angle);
    //    vertices[i] = vertices[i].rotate (q.getAxis (), q.getAngle ());
    //translate
    vertices[i] += pos;
  }

  return Polygon (vertices);
}

float
al::math::getDistanceBetween (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2)
{
  float d_x = v1.x - v2.x;
  float d_y = v1.y - v2.y;
  float d_z = v1.z - v2.z;

  return sqrt (d_x * d_x + d_y * d_y + d_z * d_z);
}

double
al::math::calBoxSimilarity (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2, double w_x, double w_y,
                            double w_z)
{
  //there are 6 different possibilities for a box
  //compare all six possibilities and pick the lowest one
  float difference[6];
  float v1_[3] = {v1.x, v1.y, v1.z};
  float v2_[3] = {v2.x, v2.y, v2.z};

  float dx, dy, dz;
  //all combinations calculated
  for (uint i = 0; i < 3; i++)
  {
    dx = fabs (v1_[0] - v2_[i]);
    dy = fabs (v1_[1] - v2_[(i + 1) % 3]);
    dz = fabs (v1_[2] - v2_[(i + 2) % 3]);
    difference[i] = (w_x * dx * dx + w_y * dy * dy + w_z * dz * dz) / (w_x + w_y + w_z);
  }
  for (uint i = 0; i < 3; i++)
  {
    dx = fabs (v1_[0] - v2_[i]);
    dy = fabs (v1_[1] - v2_[(i - 1) % 3]);
    dz = fabs (v1_[2] - v2_[(i - 2) % 3]);
    difference[i + 3] = (w_x * dx * dx + w_y * dy * dy + w_z * dz * dz) / (w_x + w_y + w_z);
  }
  float min_diff = difference[0];
  for (uint i = 1; i < 6; i++)
  {
    if (difference[i] < min_diff)
      min_diff = difference[i];
  }
  return min_diff;
}

double
al::math::calBoxSimilarity (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2)
{
  //there are 6 different possibilities for a box
  //get the max dimensional difference between each edge for every combination
  float difference[6];
  float v1_[3] = {v1.x, v1.y, v1.z};
  float v2_[3] = {v2.x, v2.y, v2.z};

  float dx, dy, dz;
  //all combinations calculated
  for (uint i = 0; i < 3; i++)
  {
    dx = fabs (v1_[0] - v2_[i]);
    dy = fabs (v1_[1] - v2_[(i + 1) % 3]);
    dz = fabs (v1_[2] - v2_[(i + 2) % 3]);

    difference[i] = std::max (std::max (dx, dy), dz);
    //    std::cout << dx << " " << dy << " " << dz << "\t" << difference[i] << std::endl;
  }
  for (uint i = 0; i < 3; i++)
  {
    dx = fabs (v1_[0] - v2_[i]);
    dy = fabs (v1_[1] - v2_[(i - 1) % 3]);
    dz = fabs (v1_[2] - v2_[(i - 2) % 3]);

    difference[i + 3] = std::max (std::max (dx, dy), dz);
    //    std::cout << dx << " " << dy << " " << dz << "\t" << difference[i + 3] << std::endl;
  }
  float min_diff = difference[0];
  for (uint i = 1; i < 6; i++)
  {
    if (difference[i] < min_diff)
      min_diff = difference[i];
  }
  return min_diff;
}

float
al::math::getDistanceBetween (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  float d_x = p1.x - p2.x;
  float d_y = p1.y - p2.y;
  float d_z = p1.z - p2.z;

  return sqrt (d_x * d_x + d_y * d_y + d_z * d_z);
}

btMatrix3x3
al::math::getTransformationBetween (tf::Vector3 v_src, tf::Vector3 v_dest)
{
  //transformation: v_dest * v_src^(-1)
  cv::Mat mat_src, mat_dest;
  float data[] = {v_src.x (), v_src.y (), v_src.z ()};
  mat_src = cv::Mat (3, 1, CV_32FC1, data);

  data[0] = v_dest.x ();
  data[1] = v_dest.y ();
  data[2] = v_dest.z ();
  mat_dest = cv::Mat (3, 1, CV_32FC1, data);
  cv::Mat inv;
  cv::invert (mat_src, inv, cv::DECOMP_SVD);

  cv::Mat tf;
  tf = mat_dest * inv;
  btMatrix3x3 bt_tf;
  bt_tf.setValue (tf.at<float> (0, 0), tf.at<float> (0, 1), tf.at<float> (0, 2), tf.at<float> (1, 0),
                  tf.at<float> (1, 1), tf.at<float> (1, 2), tf.at<float> (2, 0), tf.at<float> (2, 1),
                  tf.at<float> (2, 2));
  return bt_tf;
}

double
al::math::differentiate (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point, DiffMode diff_mode)
{
  uint n_rows = mat_eqn_coeffs.rows;
  uint n_cols = mat_eqn_coeffs.cols;

  //if there is no term with at least x to the power 1, then differentiation returns 0
  if (n_rows < 2 && (diff_mode == F_X || diff_mode == F_XX))
    return 0;

  //if there is no term with at least y to the power 1, then differentiation returns 0
  if (n_cols < 2 && (diff_mode == F_Y || diff_mode == F_YY))
    return 0;

  //powers decreases by one according to the f_x and f_y, but anyway
  cv::Mat tmp_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat tmp_diffed_eqn_coeffs_2 = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);

  if (diff_mode == F_X || diff_mode == F_XX || diff_mode == F_XY)
  {
    //f_x for xi=0 returns always zero
    for (uint yi = 0; yi < n_cols; yi++)
      tmp_diffed_eqn_coeffs.at<double> (0, yi) = 0;

    for (uint xi = 1; xi < n_rows; xi++)
      for (uint yi = 0; yi < n_cols; yi++)
        if (mat_eqn_coeffs.at<double> (xi, yi))
          tmp_diffed_eqn_coeffs.at<double> (xi - 1, yi) = xi * mat_eqn_coeffs.at<double> (xi, yi);

    if (diff_mode == F_XY)
    {
      //f_y for yi=0 returns always zero
//      for (uint xi = 0; xi < n_rows; xi++)
//        tmp_diffed_eqn_coeffs.at<double> (xi, 0) = 0;

      for (uint xi = 0; xi < n_rows; xi++)
        for (uint yi = 1; yi < n_cols; yi++)
          if (tmp_diffed_eqn_coeffs.at<double> (xi, yi))
            tmp_diffed_eqn_coeffs_2.at<double> (xi, yi - 1) = yi * tmp_diffed_eqn_coeffs.at<double> (xi, yi);

      tmp_diffed_eqn_coeffs = tmp_diffed_eqn_coeffs_2;
    }
    else if (diff_mode == F_XX)
    {
      //f_x for xi=0 returns always zero
//      for (uint yi = 0; yi < n_cols; yi++)
//        tmp_diffed_eqn_coeffs.at<double> (0, yi) = 0;

      for (uint xi = 1; xi < n_rows; xi++)
        for (uint yi = 0; yi < n_cols; yi++)
          if (tmp_diffed_eqn_coeffs.at<double> (xi, yi))
            tmp_diffed_eqn_coeffs_2.at<double> (xi - 1, yi) = xi * tmp_diffed_eqn_coeffs.at<double> (xi, yi);

      tmp_diffed_eqn_coeffs = tmp_diffed_eqn_coeffs_2;
    }
  }
  else
  {
    //f_y for yi=0 returns always zero
//    for (uint yi = 0; yi < n_cols; yi++)
//      tmp_diffed_eqn_coeffs.at<double> (0, yi) = 0;

    for (uint xi = 0; xi < n_rows; xi++)
      for (uint yi = 1; yi < n_cols; yi++)
        if (mat_eqn_coeffs.at<double> (xi, yi))
          tmp_diffed_eqn_coeffs.at<double> (xi, yi - 1) = yi * mat_eqn_coeffs.at<double> (xi, yi);

    if (diff_mode == F_YY)
    {
      for (uint xi = 0; xi < n_rows; xi++)
        for (uint yi = 1; yi < n_cols; yi++)
          if (tmp_diffed_eqn_coeffs.at<double> (xi, yi))
            tmp_diffed_eqn_coeffs_2.at<double> (xi, yi - 1) = yi * tmp_diffed_eqn_coeffs.at<double> (xi, yi);

      tmp_diffed_eqn_coeffs = tmp_diffed_eqn_coeffs_2;
    }
  }

  double differentiation = 0;
  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      differentiation += tmp_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  return differentiation;
}

cv::Mat
al::math::calcHessian (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point)
{
  cv::Mat hessian = cv::Mat::zeros (2, 2, CV_64FC1);

  uint n_rows = mat_eqn_coeffs.rows;
  uint n_cols = mat_eqn_coeffs.cols;

  //powers decreases by one according to the f_x and f_y, but anyway
  cv::Mat x_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat xx_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat xy_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat y_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat yy_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);

  for (uint xi = 1; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      if (mat_eqn_coeffs.at<double> (xi, yi))
        x_diffed_eqn_coeffs.at<double> (xi - 1, yi) = xi * mat_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 1; yi < n_cols; yi++)
      if (x_diffed_eqn_coeffs.at<double> (xi, yi))
        xy_diffed_eqn_coeffs.at<double> (xi, yi - 1) = yi * x_diffed_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 1; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      if (x_diffed_eqn_coeffs.at<double> (xi, yi))
        xx_diffed_eqn_coeffs.at<double> (xi - 1, yi) = xi * x_diffed_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 1; yi < n_cols; yi++)
      if (mat_eqn_coeffs.at<double> (xi, yi))
        y_diffed_eqn_coeffs.at<double> (xi, yi - 1) = yi * mat_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 1; yi < n_cols; yi++)
      if (y_diffed_eqn_coeffs.at<double> (xi, yi))
        yy_diffed_eqn_coeffs.at<double> (xi, yi - 1) = yi * y_diffed_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      hessian.at<double> (0, 0) += xx_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      hessian.at<double> (0, 1) += xy_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  hessian.at<double> (1, 0) = hessian.at<double> (0, 1);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      hessian.at<double> (1, 1) += yy_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  return hessian;
}

cv::Mat
al::math::calcHessianSqrted (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point, double& f_x, double &f_y)
{
  uint n_rows = mat_eqn_coeffs.rows;
  uint n_cols = mat_eqn_coeffs.cols;

  double f_x_squared_eqn, f_y_squared_eqn;
  cv::Mat hessian_squared_eqn = calcHessian (mat_eqn_coeffs, point, f_x_squared_eqn, f_y_squared_eqn);

  cv::Mat hessian = cv::Mat::zeros (2, 2, CV_64FC1);

  //z fonksiyonunu hesapla bu noktada, bu eqn demek asagida
  double fnc = 0;
  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      fnc += mat_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi) * pow ((double)point.y (), (int)yi);

  f_x = f_x_squared_eqn / (2 * fnc);
  f_y = f_y_squared_eqn / (2 * fnc);

  hessian.at<double> (0, 0) = (hessian_squared_eqn.at<double> (0, 0) / 2 - f_x * f_x) / fnc;
  hessian.at<double> (1, 1) = (hessian_squared_eqn.at<double> (1, 1) / 2 - f_y * f_y) / fnc;
  hessian.at<double> (0, 1) = (hessian_squared_eqn.at<double> (0, 1) / 2 - f_x * f_y) / fnc;
  hessian.at<double> (1, 0) = hessian.at<double> (0, 1);

  return hessian;
}

cv::Mat
al::math::calcHessian (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point, double& f_x, double &f_y)
{
  cv::Mat hessian = cv::Mat::zeros (2, 2, CV_64FC1);

  uint n_rows = mat_eqn_coeffs.rows;
  uint n_cols = mat_eqn_coeffs.cols;

  //powers decreases by one according to the f_x and f_y, but anyway
  cv::Mat x_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat xx_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat xy_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat y_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);
  cv::Mat yy_diffed_eqn_coeffs = cv::Mat::zeros (n_rows, n_cols, CV_64FC1);

  for (uint xi = 1; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      if (mat_eqn_coeffs.at<double> (xi, yi))
        x_diffed_eqn_coeffs.at<double> (xi - 1, yi) = xi * mat_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 1; yi < n_cols; yi++)
      if (x_diffed_eqn_coeffs.at<double> (xi, yi))
        xy_diffed_eqn_coeffs.at<double> (xi, yi - 1) = yi * x_diffed_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 1; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      if (x_diffed_eqn_coeffs.at<double> (xi, yi))
        xx_diffed_eqn_coeffs.at<double> (xi - 1, yi) = xi * x_diffed_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 1; yi < n_cols; yi++)
      if (mat_eqn_coeffs.at<double> (xi, yi))
        y_diffed_eqn_coeffs.at<double> (xi, yi - 1) = yi * mat_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 1; yi < n_cols; yi++)
      if (y_diffed_eqn_coeffs.at<double> (xi, yi))
        yy_diffed_eqn_coeffs.at<double> (xi, yi - 1) = yi * y_diffed_eqn_coeffs.at<double> (xi, yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      hessian.at<double> (0, 0) += xx_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      hessian.at<double> (0, 1) += xy_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  hessian.at<double> (1, 0) = hessian.at<double> (0, 1);

  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      hessian.at<double> (1, 1) += yy_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  f_x = 0;
  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      f_x += x_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  f_y = 0;
  for (uint xi = 0; xi < n_rows; xi++)
    for (uint yi = 0; yi < n_cols; yi++)
      f_y += y_diffed_eqn_coeffs.at<double> (xi, yi) * pow ((double)point.x (), (int)xi)
          * pow ((double)point.y (), (int)yi);

  return hessian;
}

bool
al::math::calcCurvatures (Curvatures &curvatures, const cv::Mat &hessian, const double f_x, const double f_y)
{
  double h_u = f_x, h_v = f_y;
  double h_uu = hessian.at<double> (0, 0);
  double h_vv = hessian.at<double> (1, 1);
  double h_uv = hessian.at<double> (0, 1);

  double E = 1 + h_u * h_u;
  double F = h_u * h_v;
  double G = 1 + h_v * h_v;

  double h_u_squared = h_u * h_u;
  double h_v_squared = h_v * h_v;
  double denominator = sqrt (1 + h_u_squared + h_v_squared);

  double e = h_uu / denominator;
  double f = h_uv / denominator;
  double g = h_vv / denominator;

  curvatures.H = (e * G - 2 * f * F + g * E) / (2 * (E * G - F * F));
  curvatures.K = (e * g - f * f) / (E * G - F * F);

  //k_1 is always larger than k_2
  curvatures.pc_1 = curvatures.H + sqrt (curvatures.H * curvatures.H - curvatures.K);
  curvatures.pc_2 = curvatures.H - sqrt (curvatures.H * curvatures.H - curvatures.K);

  curvatures.C = sqrt ((curvatures.pc_1 * curvatures.pc_1 + curvatures.pc_2 * curvatures.pc_2) / 2);

  if (curvatures.pc_1 != curvatures.pc_2)
    curvatures.S = atan ((curvatures.pc_1 + curvatures.pc_2) / (curvatures.pc_2 - curvatures.pc_1)) * 2 / M_PI;
  else
  {
    if (curvatures.pc_1 == 0 && curvatures.pc_2 == 0)
      curvatures.S = NAN;
    else
      curvatures.S = 0;
  }

  return true;
}

bool
al::math::calcCurvatures (Curvatures &curvatures, const Eigen::MatrixXd &weingarten_matrix)
{
//  std::cout << "eigen values: " << weingarten_matrix.eigenvalues () << std::endl;
//  std::cout << weingarten.eigenvalues ().data ()[0] << "\t" << weingarten.eigenvalues ().data ()[1] << std::endl;
//  std::cout << weingarten.eigenvalues ().data ()[0].real () << "\t" << weingarten.eigenvalues ().data ()[1].real ()
//      << std::endl;
//  std::cout << weingarten.eigenvalues ().data ()[0].imag () << "\t" << weingarten.eigenvalues ().data ()[1].imag ()
//      << std::endl;

  if (weingarten_matrix.eigenvalues ().data ()[0].real () > weingarten_matrix.eigenvalues ().data ()[1].real ())
  {
    curvatures.pc_1 = weingarten_matrix.eigenvalues ().data ()[0].real ();
    curvatures.pc_2 = weingarten_matrix.eigenvalues ().data ()[1].real ();
  }
  else
  {
    curvatures.pc_1 = weingarten_matrix.eigenvalues ().data ()[1].real ();
    curvatures.pc_2 = weingarten_matrix.eigenvalues ().data ()[0].real ();
  }

  curvatures.C = sqrt ((curvatures.pc_1 * curvatures.pc_1 + curvatures.pc_2 * curvatures.pc_2) / 2);

  if (curvatures.pc_1 != curvatures.pc_2)
    curvatures.S = atan ((curvatures.pc_1 + curvatures.pc_2) / (curvatures.pc_2 - curvatures.pc_1)) * 2 / M_PI;
  else
  {
    if (curvatures.pc_1 == 0 && curvatures.pc_2 == 0)
      curvatures.S = NAN;
    else
      curvatures.S = 0;
  }

  curvatures.K = curvatures.pc_1 * curvatures.pc_2;
  curvatures.H = (curvatures.pc_1 + curvatures.pc_2) / 2;

  return true;
}

///*

bool
al::math::calcPointPCurvs (pcl::PrincipalCurvatures &pcurvs, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data,
                           pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                           pcl::search::KdTree<pcl::PointXYZ>::Ptr tree, const int pt_idx,
                           const int curv_estimation_method, const int n_neighbors)
{
  std::vector<int> neighbor_ids (n_neighbors);
  std::vector<float> pointsSquaredDist (n_neighbors);
  tree->nearestKSearch (pt_idx, n_neighbors, neighbor_ids, pointsSquaredDist);

  if (curv_estimation_method == GOLDFEATHER2004)
  {
    std::map<int, pcl::PointXYZ> map_id_to_pt;
    for (uint i = 1; i < neighbor_ids.size (); i++)
      map_id_to_pt[neighbor_ids[i]] = pointcloud_data->points[neighbor_ids[i]];

    //transform neighbor points to local (e.g. the coordinate system located at cloud[pt_idx])
    btMatrix3x3 transformation;
    btVector3 n_i;
    transformation.setIdentity ();
    pcl::Normal normal = pointcloud_normals->points[pt_idx];
    n_i.setValue (normal.normal_x, normal.normal_y, normal.normal_z);
    btVector3 z (0, 0, 1);
    tf::Vector3 tmp_z (0, 0, 1);
    tf::Vector3 tmp_n_i (n_i.x (), n_i.y (), n_i.z ());
    float angle = al::math::getAngleBetween (tmp_n_i, tmp_z);
    btVector3 axis = n_i.cross (z);

    if (fabs (angle) > 0.05)
      transformation.setRotation (btQuaternion (axis, angle));
    else
      transformation.setIdentity ();

    transformation.setRotation (btQuaternion (axis, angle));

    btVector3 p_i (pointcloud_data->points[pt_idx].x, pointcloud_data->points[pt_idx].y,
                   pointcloud_data->points[pt_idx].z);

    for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
    {
      btVector3 p_n (pointcloud_data->points[neighbor_ids[n]].x, pointcloud_data->points[neighbor_ids[n]].y,
                     pointcloud_data->points[neighbor_ids[n]].z);
      //first translate to the origin
      p_n -= p_i;
      //now rotate
      p_n = transformation * p_n;
      map_id_to_pt[neighbor_ids[n]].x = p_n.x ();
      map_id_to_pt[neighbor_ids[n]].y = p_n.y ();
      map_id_to_pt[neighbor_ids[n]].z = p_n.z ();
    }

    //NN=8, 9 including itself
    //utilizing surface normal information
    std::vector<double> point_based_eqn (7, 0.0);
    std::vector<double> norm_x_based_eqn (7, 0.0);
    std::vector<double> norm_y_based_eqn (7, 0.0);

    Eigen::MatrixXd M ((neighbor_ids.size () - 1) * 3, 7);
    Eigen::MatrixXd R ((neighbor_ids.size () - 1) * 3, 1);
    Eigen::MatrixXd U (7, 1);
    Eigen::MatrixXd W (2, 2);

    int offset_1 = neighbor_ids.size () - 1;
    int offset_2 = 2 * offset_1;

    for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
    {
      btVector3 p_n (map_id_to_pt[neighbor_ids[n]].x, map_id_to_pt[neighbor_ids[n]].y, map_id_to_pt[neighbor_ids[n]].z);
      btVector3 n_n (pointcloud_normals->points[neighbor_ids[n]].normal_x + pointcloud_data->points[neighbor_ids[n]].x,
                     pointcloud_normals->points[neighbor_ids[n]].normal_y + pointcloud_data->points[neighbor_ids[n]].y,
                     pointcloud_normals->points[neighbor_ids[n]].normal_z + pointcloud_data->points[neighbor_ids[n]].z);

      n_n = n_n - p_i;

      n_n = transformation * n_n - p_n;
      M (n - 1, 0) = p_n.x () * p_n.x () * 0.5;
      M (n - 1, 1) = p_n.x () * p_n.y ();
      M (n - 1, 2) = p_n.y () * p_n.y () * 0.5;
      M (n - 1, 3) = p_n.x () * p_n.x () * p_n.x ();
      M (n - 1, 4) = p_n.x () * p_n.x () * p_n.y ();
      M (n - 1, 5) = p_n.x () * p_n.y () * p_n.y ();
      M (n - 1, 6) = p_n.y () * p_n.y () * p_n.y ();

      M (offset_1 + n - 1, 0) = p_n.x ();
      M (offset_1 + n - 1, 1) = p_n.y ();
      M (offset_1 + n - 1, 2) = 0.0;
      M (offset_1 + n - 1, 3) = p_n.x () * p_n.x () * 3;
      M (offset_1 + n - 1, 4) = p_n.x () * p_n.y () * 2;
      M (offset_1 + n - 1, 5) = p_n.y () * p_n.y ();
      M (offset_1 + n - 1, 6) = 0.0;

      M (offset_2 + n - 1, 0) = 0.0;
      M (offset_2 + n - 1, 1) = p_n.x ();
      M (offset_2 + n - 1, 2) = p_n.y ();
      M (offset_2 + n - 1, 3) = 0.0;
      M (offset_2 + n - 1, 4) = p_n.x () * p_n.x ();
      M (offset_2 + n - 1, 5) = 2 * p_n.x () * p_n.y ();
      M (offset_2 + n - 1, 6) = p_n.y () * p_n.y () * 3;

      R (n - 1, 0) = p_n.z ();
      R (offset_1 + n - 1, 0) = -n_n.x () / n_n.z ();
      R (offset_2 + n - 1, 0) = -n_n.y () / n_n.z ();
    }

    //apply least squares
    U = ((M.transpose () * M).inverse () * M.transpose ()) * R;

    W (0, 0) = U (0, 0); //a
    W (0, 1) = U (1, 0); //b
    W (1, 0) = U (1, 0); //b
    W (1, 1) = U (2, 0); //c

    if (W.eigenvalues ().data ()[0].real () > W.eigenvalues ().data ()[1].real ())
    {
      pcurvs.pc1 = W.eigenvalues ().data ()[0].real ();
      pcurvs.pc2 = W.eigenvalues ().data ()[1].real ();
    }
    else
    {
      pcurvs.pc1 = W.eigenvalues ().data ()[1].real ();
      pcurvs.pc2 = W.eigenvalues ().data ()[0].real ();
    }
  }
  else if (curv_estimation_method == QUADRIC_FITTING)
  {
    const std::vector<double> mat_eqn_coeffs = fitQuadricSurface (pointcloud_data, pt_idx, neighbor_ids,
                                                                  pointcloud_normals);
    tf::Vector3 point (0, 0, 0);
    cv::Mat hessian;
    double f_x, f_y;

    cv::Mat fnc = cv::Mat::zeros (3, 3, CV_64FC1);
    for (uint j = 0; j < mat_eqn_coeffs.size (); j++)
      fnc.at<double> (j / 3, j % 3) = mat_eqn_coeffs[j];

    hessian = al::math::calcHessian (fnc, point, f_x, f_y);

    double h_u = f_x, h_v = f_y;
    double h_uu = hessian.at<double> (0, 0);
    double h_vv = hessian.at<double> (1, 1);
    double h_uv = hessian.at<double> (0, 1);

    double E = 1 + h_u * h_u;
    double F = h_u * h_v;
    double G = 1 + h_v * h_v;

    double h_u_squared = h_u * h_u;
    double h_v_squared = h_v * h_v;
    double denominator = sqrt (1 + h_u_squared + h_v_squared);

    double e = h_uu / denominator;
    double f = h_uv / denominator;
    double g = h_vv / denominator;

    double mean_curv = (e * G - 2 * f * F + g * E) / (2 * (E * G - F * F));
    double gaus_curv = (e * g - f * f) / (E * G - F * F);

    //k_1 is always larger than k_2
    pcurvs.pc1 = mean_curv + sqrt (mean_curv * mean_curv - gaus_curv);
    pcurvs.pc2 = mean_curv - sqrt (mean_curv * mean_curv - gaus_curv);

  }
  else
    return false;

  return true;
}

bool
al::math::calcPointPCurvs (pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr pointcloud_data,
                           pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                           pcl::search::KdTree<pcl::PointXYZPCurvs>::Ptr tree, const int pt_idx,
                           const int curv_estimation_method, const int n_neighbors)
{
  std::vector<int> neighbor_ids (n_neighbors);
  std::vector<float> pointsSquaredDist (n_neighbors);
  tree->nearestKSearch (pt_idx, n_neighbors, neighbor_ids, pointsSquaredDist);
  //  std::cout << "pt_idx" << pt_idx << "\t" << pointcloud_normals->points[pt_idx] << "\t";
  //  for (uint i = 0; i < n_neighbors; i++)
  //    std::cout << neighbor_ids[i] << " ";
  //  std::cout << std::endl;

  if (curv_estimation_method == GOLDFEATHER2004)
  {
    std::map<int, pcl::PointXYZPCurvs> map_id_to_pt;
    for (uint i = 1; i < neighbor_ids.size (); i++)
      map_id_to_pt[neighbor_ids[i]] = pointcloud_data->points[neighbor_ids[i]];

    //transform neighbor points to local (e.g. the coordinate system located at cloud[pt_idx])
    btMatrix3x3 transformation;
    btVector3 n_i;
    transformation.setIdentity ();
    pcl::Normal normal = pointcloud_normals->points[pt_idx];
    n_i.setValue (normal.normal_x, normal.normal_y, normal.normal_z);
    btVector3 z (0, 0, 1);

    float angle = al::math::getAngleBetween (n_i, z);
    btVector3 axis = n_i.cross (z);

    if (fabs (angle) > 0.05)
      transformation.setRotation (btQuaternion (axis, angle));
    else
      transformation.setIdentity ();

    transformation.setRotation (btQuaternion (axis, angle));

    btVector3 p_i (pointcloud_data->points[pt_idx].x, pointcloud_data->points[pt_idx].y,
                   pointcloud_data->points[pt_idx].z);

    for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
    {
      btVector3 p_n (pointcloud_data->points[neighbor_ids[n]].x, pointcloud_data->points[neighbor_ids[n]].y,
                     pointcloud_data->points[neighbor_ids[n]].z);
      //first translate to the origin
      p_n -= p_i;
      //now rotate
      p_n = transformation * p_n;
      map_id_to_pt[neighbor_ids[n]].x = p_n.x ();
      map_id_to_pt[neighbor_ids[n]].y = p_n.y ();
      map_id_to_pt[neighbor_ids[n]].z = p_n.z ();
    }

    //NN=8, 9 including itself
    //utilizing surface normal information
    std::vector<double> point_based_eqn (7, 0.0);
    std::vector<double> norm_x_based_eqn (7, 0.0);
    std::vector<double> norm_y_based_eqn (7, 0.0);

    Eigen::MatrixXd M ((neighbor_ids.size () - 1) * 3, 7);
    Eigen::MatrixXd R ((neighbor_ids.size () - 1) * 3, 1);
    Eigen::MatrixXd U (7, 1);
    Eigen::MatrixXd W (2, 2);

    int offset_1 = neighbor_ids.size () - 1;
    int offset_2 = 2 * offset_1;

    for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
    {
      btVector3 p_n (map_id_to_pt[neighbor_ids[n]].x, map_id_to_pt[neighbor_ids[n]].y, map_id_to_pt[neighbor_ids[n]].z);
      btVector3 n_n (pointcloud_normals->points[neighbor_ids[n]].normal_x + pointcloud_data->points[neighbor_ids[n]].x,
                     pointcloud_normals->points[neighbor_ids[n]].normal_y + pointcloud_data->points[neighbor_ids[n]].y,
                     pointcloud_normals->points[neighbor_ids[n]].normal_z + pointcloud_data->points[neighbor_ids[n]].z);

      n_n = n_n - p_i;

      n_n = transformation * n_n - p_n;
      M (n - 1, 0) = p_n.x () * p_n.x () * 0.5;
      M (n - 1, 1) = p_n.x () * p_n.y ();
      M (n - 1, 2) = p_n.y () * p_n.y () * 0.5;
      M (n - 1, 3) = p_n.x () * p_n.x () * p_n.x ();
      M (n - 1, 4) = p_n.x () * p_n.x () * p_n.y ();
      M (n - 1, 5) = p_n.x () * p_n.y () * p_n.y ();
      M (n - 1, 6) = p_n.y () * p_n.y () * p_n.y ();

      M (offset_1 + n - 1, 0) = p_n.x ();
      M (offset_1 + n - 1, 1) = p_n.y ();
      M (offset_1 + n - 1, 2) = 0.0;
      M (offset_1 + n - 1, 3) = p_n.x () * p_n.x () * 3;
      M (offset_1 + n - 1, 4) = p_n.x () * p_n.y () * 2;
      M (offset_1 + n - 1, 5) = p_n.y () * p_n.y ();
      M (offset_1 + n - 1, 6) = 0.0;

      M (offset_2 + n - 1, 0) = 0.0;
      M (offset_2 + n - 1, 1) = p_n.x ();
      M (offset_2 + n - 1, 2) = p_n.y ();
      M (offset_2 + n - 1, 3) = 0.0;
      M (offset_2 + n - 1, 4) = p_n.x () * p_n.x ();
      M (offset_2 + n - 1, 5) = 2 * p_n.x () * p_n.y ();
      M (offset_2 + n - 1, 6) = p_n.y () * p_n.y () * 3;

      R (n - 1, 0) = p_n.z ();
      R (offset_1 + n - 1, 0) = -n_n.x () / n_n.z ();
      R (offset_2 + n - 1, 0) = -n_n.y () / n_n.z ();
    }

    //apply least squares
    U = ((M.transpose () * M).inverse () * M.transpose ()) * R;

    W (0, 0) = U (0, 0); //a
    W (0, 1) = U (1, 0); //b
    W (1, 0) = U (1, 0); //b
    W (1, 1) = U (2, 0); //c

    if (W.eigenvalues ().data ()[0].real () > W.eigenvalues ().data ()[1].real ())
    {
      pointcloud_data->points[pt_idx].pc_1 = W.eigenvalues ().data ()[0].real ();
      pointcloud_data->points[pt_idx].pc_2 = W.eigenvalues ().data ()[1].real ();
    }
    else
    {
      pointcloud_data->points[pt_idx].pc_1 = W.eigenvalues ().data ()[1].real ();
      pointcloud_data->points[pt_idx].pc_2 = W.eigenvalues ().data ()[0].real ();
    }
  }
  else if (curv_estimation_method == QUADRIC_FITTING)
  {
    const std::vector<double> mat_eqn_coeffs = fitQuadricSurface (pointcloud_data, pt_idx, neighbor_ids,
                                                                  pointcloud_normals);
    tf::Vector3 point (0, 0, 0);
    cv::Mat hessian;
    double f_x, f_y;

    cv::Mat fnc = cv::Mat::zeros (3, 3, CV_64FC1);
    for (uint j = 0; j < mat_eqn_coeffs.size (); j++)
      fnc.at<double> (j / 3, j % 3) = mat_eqn_coeffs[j];

    hessian = al::math::calcHessian (fnc, point, f_x, f_y);

    double h_u = f_x, h_v = f_y;
    double h_uu = hessian.at<double> (0, 0);
    double h_vv = hessian.at<double> (1, 1);
    double h_uv = hessian.at<double> (0, 1);

    double E = 1 + h_u * h_u;
    double F = h_u * h_v;
    double G = 1 + h_v * h_v;

    double h_u_squared = h_u * h_u;
    double h_v_squared = h_v * h_v;
    double denominator = sqrt (1 + h_u_squared + h_v_squared);

    double e = h_uu / denominator;
    double f = h_uv / denominator;
    double g = h_vv / denominator;

    double mean_curv = (e * G - 2 * f * F + g * E) / (2 * (E * G - F * F));
    double gaus_curv = (e * g - f * f) / (E * G - F * F);

    //k_1 is always larger than k_2
    pointcloud_data->points[pt_idx].pc_1 = mean_curv + sqrt (mean_curv * mean_curv - gaus_curv);
    pointcloud_data->points[pt_idx].pc_2 = mean_curv - sqrt (mean_curv * mean_curv - gaus_curv);
  }
  else
    return false;

  //  std::cout << pointcloud_data->points[pt_idx].pc_1 << "\t" << pointcloud_data->points[pt_idx].pc_2 << std::endl;
  return true;
}

std::vector<double>
al::math::fitQuadricSurface (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data, const int pt_idx,
                             const std::vector<int> &neighbor_ids, pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals)
{
  std::map<int, pcl::PointXYZ> map_id_to_pt;
  for (uint i = 1; i < neighbor_ids.size (); i++)
    map_id_to_pt[neighbor_ids[i]] = pointcloud_data->points[neighbor_ids[i]];

  btMatrix3x3 transformation;
  btVector3 n_i;
  transformation.setIdentity ();
  pcl::Normal normal = pointcloud_normals->points[pt_idx];
  n_i.setValue (normal.normal_x, normal.normal_y, normal.normal_z);
  btVector3 z (0, 0, 1);
  float angle = al::math::getAngleBetween (n_i, z);
  btVector3 axis = n_i.cross (z);

  if (fabs (angle) > 0.05)
    transformation.setRotation (btQuaternion (axis, angle));
  else
    transformation.setIdentity ();

  btVector3 p_i (pointcloud_data->points[pt_idx].x, pointcloud_data->points[pt_idx].y,
                 pointcloud_data->points[pt_idx].z);

  for (uint n = 0; n < (uint)neighbor_ids.size (); n++)
  {
    btVector3 p_n (pointcloud_data->points[neighbor_ids[n]].x, pointcloud_data->points[neighbor_ids[n]].y,
                   pointcloud_data->points[neighbor_ids[n]].z);
    //first translate to the origin
    p_n -= p_i;
    //now rotate
    p_n = transformation * p_n;
    map_id_to_pt[neighbor_ids[n]].x = p_n.x ();
    map_id_to_pt[neighbor_ids[n]].y = p_n.y ();
    map_id_to_pt[neighbor_ids[n]].z = p_n.z ();
  }

  int x_row_pow = 0, x_col_pow = 0, y_row_pow = 0, y_col_pow = 0;
  Eigen::Matrix<double, 9, 9> D, D_inv;
  D.setZero (9, 9);
  Eigen::Matrix<double, 9, 1> A, Z;
  Z.setZero (9, 1);
  pcl::PointXYZ p_n;

  for (uint n = 0; n < (uint)neighbor_ids.size (); n++)
  {
    p_n = map_id_to_pt[neighbor_ids[n]];
    for (uint i = 0; i < 9; i++)
    {
      x_row_pow = i / 3;
      y_row_pow = i % 3;

      Z (i, 0) = Z (i, 0) + p_n.z * pow (p_n.x, x_row_pow) * pow (p_n.y, y_row_pow);

      for (uint j = 0; j < 9; j++)
      {
        x_col_pow = j / 3;
        y_col_pow = j % 3;
        D (i, j) = D (i, j) + pow (p_n.x, x_row_pow + x_col_pow) * pow (p_n.y, y_row_pow + y_col_pow);
      }
    }
  }
  D_inv = D.inverse ();
  A = D_inv * Z;

  std::vector<double> quadric_surface_params (9);
  for (uint i = 0; i < 9; i++)
    quadric_surface_params[i] = A (i, 0);

  return quadric_surface_params;
}

std::vector<double>
al::math::fitQuadricSurface (pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr pointcloud_data, const int pt_idx,
                             const std::vector<int> &neighbor_ids, pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals)
{
  std::map<int, pcl::PointXYZPCurvs> map_id_to_pt;
  for (uint i = 1; i < neighbor_ids.size (); i++)
    map_id_to_pt[neighbor_ids[i]] = pointcloud_data->points[neighbor_ids[i]];

  btMatrix3x3 transformation;
  btVector3 n_i;
  transformation.setIdentity ();
  pcl::Normal normal = pointcloud_normals->points[pt_idx];
  n_i.setValue (normal.normal_x, normal.normal_y, normal.normal_z);
  btVector3 z (0, 0, 1);
  float angle = al::math::getAngleBetween (n_i, z);
  btVector3 axis = n_i.cross (z);

  if (fabs (angle) > 0.05)
    transformation.setRotation (btQuaternion (axis, angle));
  else
    transformation.setIdentity ();

  btVector3 p_i (pointcloud_data->points[pt_idx].x, pointcloud_data->points[pt_idx].y,
                 pointcloud_data->points[pt_idx].z);

  for (uint n = 0; n < (uint)neighbor_ids.size (); n++)
  {
    btVector3 p_n (pointcloud_data->points[neighbor_ids[n]].x, pointcloud_data->points[neighbor_ids[n]].y,
                   pointcloud_data->points[neighbor_ids[n]].z);
    //first translate to the origin
    p_n -= p_i;
    //now rotate
    p_n = transformation * p_n;
    map_id_to_pt[neighbor_ids[n]].x = p_n.x ();
    map_id_to_pt[neighbor_ids[n]].y = p_n.y ();
    map_id_to_pt[neighbor_ids[n]].z = p_n.z ();
  }

  int x_row_pow = 0, x_col_pow = 0, y_row_pow = 0, y_col_pow = 0;
  Eigen::Matrix<double, 9, 9> D, D_inv;
  D.setZero (9, 9);
  Eigen::Matrix<double, 9, 1> A, Z;
  Z.setZero (9, 1);
  pcl::PointXYZPCurvs p_n;

  for (uint n = 0; n < (uint)neighbor_ids.size (); n++)
  {
    p_n = map_id_to_pt[neighbor_ids[n]];
    for (uint i = 0; i < 9; i++)
    {
      x_row_pow = i / 3;
      y_row_pow = i % 3;

      Z (i, 0) = Z (i, 0) + p_n.z * pow (p_n.x, x_row_pow) * pow (p_n.y, y_row_pow);

      for (uint j = 0; j < 9; j++)
      {
        x_col_pow = j / 3;
        y_col_pow = j % 3;
        D (i, j) = D (i, j) + pow (p_n.x, x_row_pow + x_col_pow) * pow (p_n.y, y_row_pow + y_col_pow);
      }
    }
  }
  D_inv = D.inverse ();
  A = D_inv * Z;

  std::vector<double> quadric_surface_params (9);
  for (uint i = 0; i < 9; i++)
    quadric_surface_params[i] = A (i, 0);

  return quadric_surface_params;
}

Eigen::MatrixXd
al::math::calcWeingartenMatrix (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data, const int pt_idx,
                                const std::vector<int> &neighbor_ids,
                                pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals)
{
  std::map<int, pcl::PointXYZ> map_id_to_pt;
  for (uint i = 1; i < neighbor_ids.size (); i++)
    map_id_to_pt[neighbor_ids[i]] = pointcloud_data->points[neighbor_ids[i]];

  //  pcl::PointCloud<pcl::PointXYZ> tmp_data = *pointcloud_data;

  btMatrix3x3 transformation;
  btVector3 n_i;
  transformation.setIdentity ();
  pcl::Normal normal = pointcloud_normals->points[pt_idx];
  n_i.setValue (normal.normal_x, normal.normal_y, normal.normal_z);
  btVector3 z (0, 0, 1);
  //  btVector3 z (0, 0, -1);
  float angle = al::math::getAngleBetween (n_i, z);
  btVector3 axis = n_i.cross (z);

  if (fabs (angle) > 0.05)
    transformation.setRotation (btQuaternion (axis, angle));
  else
    transformation.setIdentity ();

  transformation.setRotation (btQuaternion (axis, angle));

  btVector3 p_i (pointcloud_data->points[pt_idx].x, pointcloud_data->points[pt_idx].y,
                 pointcloud_data->points[pt_idx].z);

  for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
  {
    btVector3 p_n (pointcloud_data->points[neighbor_ids[n]].x, pointcloud_data->points[neighbor_ids[n]].y,
                   pointcloud_data->points[neighbor_ids[n]].z);
    //first translate to the origin
    p_n -= p_i;
    //now rotate
    p_n = transformation * p_n;
    //    tmp_data.points[neighbor_ids[n]].x = p_n.x ();
    //    tmp_data.points[neighbor_ids[n]].y = p_n.y ();
    //    tmp_data.points[neighbor_ids[n]].z = p_n.z ();
    map_id_to_pt[neighbor_ids[n]].x = p_n.x ();
    map_id_to_pt[neighbor_ids[n]].y = p_n.y ();
    map_id_to_pt[neighbor_ids[n]].z = p_n.z ();
  }

  //NN=8, 9 including itself
  //utilizing surface normal information
  std::vector<double> point_based_eqn (7, 0.0);
  std::vector<double> norm_x_based_eqn (7, 0.0);
  std::vector<double> norm_y_based_eqn (7, 0.0);

  Eigen::MatrixXd M ((neighbor_ids.size () - 1) * 3, 7);
  Eigen::MatrixXd R ((neighbor_ids.size () - 1) * 3, 1);
  Eigen::MatrixXd U (7, 1);
  Eigen::MatrixXd W (2, 2);

  int offset_1 = neighbor_ids.size () - 1;
  int offset_2 = 2 * offset_1;

  for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
  {
    //    btVector3 p_n (tmp_data.points[neighbor_ids[n]].x, tmp_data.points[neighbor_ids[n]].y,
    //                   tmp_data.points[neighbor_ids[n]].z);
    btVector3 p_n (map_id_to_pt[neighbor_ids[n]].x, map_id_to_pt[neighbor_ids[n]].y, map_id_to_pt[neighbor_ids[n]].z);
    btVector3 n_n (pointcloud_normals->points[neighbor_ids[n]].normal_x + pointcloud_data->points[neighbor_ids[n]].x,
                   pointcloud_normals->points[neighbor_ids[n]].normal_y + pointcloud_data->points[neighbor_ids[n]].y,
                   pointcloud_normals->points[neighbor_ids[n]].normal_z + pointcloud_data->points[neighbor_ids[n]].z);

    //    std::cout << "point  :" << pointcloud_data->points[neighbor_ids[n]].x << " "
    //        << pointcloud_data->points[neighbor_ids[n]].y << " " << pointcloud_data->points[neighbor_ids[n]].z << "\n";

    //    std::cout << "normal :" << pointcloud_normals->points[neighbor_ids[n]].normal_x << " "
    //        << pointcloud_normals->points[neighbor_ids[n]].normal_y << " "
    //        << pointcloud_normals->points[neighbor_ids[n]].normal_z << "\n";

    //    std::cout << "pt+norm:" << n_n.x () << " " << n_n.y () << " " << n_n.z () << "\n";

    //    std::cout << "tfed_pt:" << p_n.x () << " " << p_n.y () << " " << p_n.z () << "\n";

    n_n = n_n - p_i;

    n_n = transformation * n_n - p_n;

    //    std::cout << "tfed_nm:" << n_n.x () << " " << n_n.y () << " " << n_n.z () << "\n";

    ////    Rewrite the normal as (− ai/ci , − bi/ci , −1).
    //Rewrite the normal as (-ai/ci , -bi/ci , 1). //upward with normal (ai, bi, ci)
    //    int weight = (-1) / n_n.z ();
    //    n_n.setX (n_n.x () * weight);
    //    n_n.setY (n_n.y () * weight);
    ////    n_n.setZ (-1);
    //    n_n.setZ (1);

    //    std::cout << (int)(n - 1) << " ";
    M (n - 1, 0) = p_n.x () * p_n.x () * 0.5;
    //    std::cout << p_n.x () * p_n.x () * 0.5 << std::endl;
    //    std::cout << M (n - 1, 0) << std::endl;
    M (n - 1, 1) = p_n.x () * p_n.y ();
    //    std::cout << p_n.x () * p_n.y () << std::endl;
    //    std::cout << M (n - 1, 1) << std::endl;
    M (n - 1, 2) = p_n.y () * p_n.y () * 0.5;
    //    std::cout << M (n - 1, 2) << std::endl;
    M (n - 1, 3) = p_n.x () * p_n.x () * p_n.x ();
    //    std::cout << M (n - 1, 3) << std::endl;
    M (n - 1, 4) = p_n.x () * p_n.x () * p_n.y ();
    //    std::cout << M (n - 1, 4) << std::endl;
    M (n - 1, 5) = p_n.x () * p_n.y () * p_n.y ();
    //    std::cout << M (n - 1, 5) << std::endl;
    M (n - 1, 6) = p_n.y () * p_n.y () * p_n.y ();
    //    std::cout << M (n - 1, 6) << std::endl;

    //    std::cout << (int)(offset_1 + n - 1) << " ";
    M (offset_1 + n - 1, 0) = p_n.x ();
    //    std::cout << M (offset_1 + n - 1, 0) << std::endl;
    M (offset_1 + n - 1, 1) = p_n.y ();
    //    std::cout << M (offset_1 + n - 1, 1) << std::endl;
    M (offset_1 + n - 1, 2) = 0.0;
    //    std::cout << M (offset_1 + n - 1, 2) << std::endl;
    M (offset_1 + n - 1, 3) = p_n.x () * p_n.x () * 3;
    //    std::cout << M (offset_1 + n - 1, 3) << std::endl;
    M (offset_1 + n - 1, 4) = p_n.x () * p_n.y () * 2;
    //    std::cout << M (offset_1 + n - 1, 4) << std::endl;
    M (offset_1 + n - 1, 5) = p_n.y () * p_n.y ();
    //    std::cout << M (offset_1 + n - 1, 5) << std::endl;
    M (offset_1 + n - 1, 6) = 0.0;
    //    std::cout << M (offset_1 + n - 1, 6) << std::endl;

    //    std::cout << (int)(offset_2 + n - 1) << "\n";
    M (offset_2 + n - 1, 0) = 0.0;
    //    std::cout << M (offset_2 + n - 1, 0) << std::endl;
    M (offset_2 + n - 1, 1) = p_n.x ();
    //    std::cout << M (offset_2 + n - 1, 1) << std::endl;
    M (offset_2 + n - 1, 2) = p_n.y ();
    //    std::cout << M (offset_2 + n - 1, 2) << std::endl;
    M (offset_2 + n - 1, 3) = 0.0;
    //    std::cout << M (offset_2 + n - 1, 3) << std::endl;
    M (offset_2 + n - 1, 4) = p_n.x () * p_n.x ();
    //    std::cout << M (offset_2 + n - 1, 4) << std::endl;
    M (offset_2 + n - 1, 5) = 2 * p_n.x () * p_n.y ();
    //    std::cout << M (offset_2 + n - 1, 5) << std::endl;
    M (offset_2 + n - 1, 6) = p_n.y () * p_n.y () * 3;
    //    std::cout << M (offset_2 + n - 1, 6) << std::endl;

    //    M = M * (2 / sqrt (p_n.x ()) * p_n.x () + p_n.y () * p_n.y ());
    //    R = R * (2 / sqrt (p_n.x ()) * p_n.x () + p_n.y () * p_n.y ());

    R (n - 1, 0) = p_n.z ();
    R (offset_1 + n - 1, 0) = -n_n.x () / n_n.z ();
    R (offset_2 + n - 1, 0) = -n_n.y () / n_n.z ();
  }

  //  std::cout << M << std::endl;
  //  std::cout << "--" << std::endl;
  //
  //  for (uint ri = 0; ri < M.rows (); ri++)
  //  {
  //    for (uint ci = 0; ci < M.cols (); ci++)
  //      std::cout << M (ri, ci) << " ";
  //    std::cout << std::endl;
  //  }

  //  std::cout << "--" << std::endl;

  //  std::cout << R << std::endl;
  //  std::cout << "=" << std::endl;
  //apply least squares
  U = ((M.transpose () * M).inverse () * M.transpose ()) * R;
  //  std::cout << U << std::endl;

  W (0, 0) = U (0, 0); //a
  W (0, 1) = U (1, 0); //b
  W (1, 0) = U (1, 0); //b
  W (1, 1) = U (2, 0); //c

  return W;
}

std::vector<al::math::Curvatures>
al::math::calcCloudCurvatures (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data, const std::vector<int> &indices,
                               const cv::Mat &mat_eqn_coeffs)
{
  tf::Vector3 point (0, 0, 0);
  cv::Mat hessian;
  double f_x, f_y;
  std::vector<al::math::Curvatures> cloud_curvs (indices.size ());

  for (uint i = 0; i < indices.size (); i++)
  {
    point.setX (pointcloud_data->points[indices[i]].x);
    point.setY (pointcloud_data->points[indices[i]].y);
    point.setZ (pointcloud_data->points[indices[i]].z);

    hessian = al::math::calcHessian (mat_eqn_coeffs, point, f_x, f_y);
    al::math::calcCurvatures (cloud_curvs[i], hessian, f_x, f_y);
  }
  return cloud_curvs;
}

std::vector<al::math::Curvatures>
al::math::calcCloudCurvatures (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data,
                               pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                               pcl::search::KdTree<pcl::PointXYZ>::Ptr tree, const std::vector<int> &indices,
                               const int n_neighbors, const int curv_estimation_method)
{
  std::vector<al::math::Curvatures> cloud_curvs (indices.size ());

  tf::Vector3 point (0, 0, 0);
  std::vector<int> neighbor_ids (n_neighbors);
  std::vector<float> pointsSquaredDist (n_neighbors);
  for (uint i = 0; i < indices.size (); i++)
  {
    tree->nearestKSearch ((int)i, n_neighbors, neighbor_ids, pointsSquaredDist);

    if (curv_estimation_method == GOLDFEATHER2004)
    {
      Eigen::MatrixXd wein = al::math::calcWeingartenMatrix (pointcloud_data, indices[i], neighbor_ids,
                                                             pointcloud_normals);
      al::math::calcCurvatures (cloud_curvs[i], wein);

    }
    else if (curv_estimation_method == QUADRIC_FITTING)
    {
      std::vector<double> params = al::math::fitQuadricSurface (pointcloud_data, indices[i], neighbor_ids,
                                                                pointcloud_normals);
      cv::Mat fnc = cv::Mat::zeros (3, 3, CV_64FC1);
      for (uint j = 0; j < params.size (); j++)
        fnc.at<double> (j / 3, j % 3) = params[j];

      double f_x, f_y;
      cv::Mat hessian = al::math::calcHessian (fnc, point, f_x, f_y);
      al::math::calcCurvatures (cloud_curvs[i], hessian, f_x, f_y);
    }
  }

  return cloud_curvs;
}

bool
al::math::calcCloudPCurvs (pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr pointcloud_data,
                           pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                           pcl::search::KdTree<pcl::PointXYZPCurvs>::Ptr tree, const int n_neighbors,
                           const int curv_estimation_method)
{
  for (uint i = 0; i < pointcloud_data->points.size (); i++)
  {
    if (!calcPointPCurvs (pointcloud_data, pointcloud_normals, tree, i, curv_estimation_method, n_neighbors))
      return false;
  }
  return true;
}

std::vector<std_msgs::ColorRGBA>
al::math::labelCurvatures (std::vector<Curvatures> &all_curvatures, int label_space, float sampling_ratio)
{
  std::vector<std_msgs::ColorRGBA> labeled_all_curvatures (all_curvatures.size ());
  for (uint i = 0; i < all_curvatures.size (); i++)
    labeled_all_curvatures[i] = labelCurvature (all_curvatures[i], label_space, sampling_ratio);

  return labeled_all_curvatures;
}

std_msgs::ColorRGBA
al::math::labelCurvature (al::math::Curvatures &curvatures, int label_space, float sampling_ratio)
{
  const double c_zero = C_ZERO * (0.0005 / sampling_ratio);
  const double h_zero = c_zero;
  const double k_zero = h_zero * h_zero;

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;

  if (label_space == HKSC)
  {
    //    if (curvatures.C > c_zero)
    //    {
    //      if (curvatures.H < -h_zero)
    //      {
    //        if (curvatures.K >= k_zero && curvatures.S >= 5.0 / 8 && curvatures.S <= 1) //BLUE
    //        {
    //          color.r = 0.0;
    //          color.g = 0.0;
    //          color.b = 1.0;
    //        }
    //        else if (fabs (curvatures.K) < k_zero && curvatures.S >= 3.0 / 8 && curvatures.S <= 5.0 / 8) //MAGENTA
    //        {
    //          color.r = 1.0;
    //          color.g = 0.0;
    //          color.b = 1.0;
    //        }
    //        else if (curvatures.K < -k_zero && curvatures.S >= 3.0 / 16 && curvatures.S <= 3.0 / 8) //RED
    //        {
    //          color.r = 1.0;
    //          color.g = 0.0;
    //          color.b = 0.0;
    //        }
    //      }
    //      else if (curvatures.H < h_zero)
    //      {
    //        if (curvatures.K < -k_zero && curvatures.S >= -3.0 / 16 && curvatures.S <= 3.0 / 16) //ORANGE
    //        {
    //          color.r = 1.0;
    //          color.g = 0.5;
    //          color.b = 0.0;
    //        }
    //      }
    //      else if (curvatures.H >= h_zero)
    //      {
    //        if (curvatures.K >= k_zero && curvatures.S >= -1 && curvatures.S <= -5.0 / 8) //CYAN
    //        {
    //          color.r = 0.0;
    //          color.g = 1.0;
    //          color.b = 1.0;
    //        }
    //        else if (fabs (curvatures.K) < k_zero && curvatures.S >= -5.0 / 8 && curvatures.S <= -3.0 / 8) //GREEN
    //        {
    //          color.r = 0.0;
    //          color.g = 1.0;
    //          color.b = 0.0;
    //        }
    //        else if (curvatures.K < -k_zero && curvatures.S >= -3.0 / 16 && curvatures.S <= 3.0 / 16) //YELLOW
    //        {
    //          color.r = 1.0;
    //          color.g = 1.0;
    //          color.b = 0.0;
    //        }
    //      }
    //    }
    //    else //PLANAR
    //    {
    //      if (curvatures.H < h_zero && curvatures.K < k_zero) //GREY
    //      {
    //        color.r = 0.5;
    //        color.g = 0.5;
    //        color.b = 0.5;
    //      }
    //    }
  }
  else if (label_space == HK)
  {
    if (curvatures.H < -h_zero)
    {
      if (curvatures.K > k_zero)
      {
        color.r = 0;
        color.g = 0;
        color.b = 1;
      }
      else if (fabs (curvatures.K) <= k_zero)
      { //magenta
        color.r = 1;
        color.g = 0;
        color.b = 1;
      }
      else
      { //red
        color.r = 1;
        color.g = 0;
        color.b = 0;
      }
    }
    else if (fabs (curvatures.H) <= h_zero)
    {
      if (curvatures.K > k_zero)
      {
        std::cerr << "impossible curvature calculation!\n";
      }
      else if (fabs (curvatures.K) <= k_zero)
      { //grey
        color.r = 0.5;
        color.g = 0.5;
        color.b = 0.5;
      }
      else
      {
        color.r = 1.0;
        color.g = 0.5;
        color.b = 0.0;
      }
    }
    else
    {
      if (curvatures.K > k_zero)
      {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
      }
      else if (fabs (curvatures.K) <= k_zero)
      {
        color.r = 0;
        color.g = 1;
        color.b = 0;
      }
      else
      {
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
      }
    }
  }
  else if (label_space == SC)
  {
    if (curvatures.C > c_zero)
    {
      if (curvatures.S >= 5.0 / 8 && curvatures.S <= 1) //BLUE
      {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
      }
      else if (curvatures.S >= 3.0 / 8 && curvatures.S < 5.0 / 8) //MAGENTA
      {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
      }
      //      else if (curvatures.S >= 3.0 / 64 && curvatures.S < 3.0 / 8) //RED
      //      {
      //        color.r = 1.0;
      //        color.g = 0.0;
      //        color.b = 0.0;
      //      }
      //      else if (curvatures.S >= -3.0 / 64 && curvatures.S < 3.0 / 64) //ORANGE
      //      {
      //        color.r = 1.0;
      //        color.g = 0.5;
      //        color.b = 0.0;
      //      }
      //      else if (curvatures.S >= -3.0 / 8 && curvatures.S < -3.0 / 64) //YELLOW
      //      {
      //        color.r = 1.0;
      //        color.g = 1.0;
      //        color.b = 0.0;
      //      }
      else if (curvatures.S >= 3.0 / 16 && curvatures.S < 3.0 / 8) //RED
      {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
      }
      else if (curvatures.S >= -3.0 / 16 && curvatures.S < 3.0 / 16) //ORANGE
      {
        color.r = 1.0;
        color.g = 0.5;
        color.b = 0.0;
      }
      else if (curvatures.S >= -3.0 / 8 && curvatures.S < -3.0 / 16) //YELLOW
      {
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
      }
      else if (curvatures.S >= -5.0 / 8 && curvatures.S < -3.0 / 8) //GREEN
      {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
      }
      else if (curvatures.S >= -1 && curvatures.S < -5.0 / 8) //CYAN
      {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
      }
    }
    else
    {
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.5;
    }
  }
  //  std::cout << color.r << " " << color.g << " " << color.b << " " << std::endl;
  return color;
}

cv::Mat
al::math::extractDepthMap (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud, const std::vector<int> &indices,
                           const tf::TransformListener &listener, const std::string sensor_frame,
                           const uint image_width)
{
  //DEV_NOTE: it doesn't make sense to transform whole cloud, since we are interested in only a part of it (object).
  //which is usally around 1000points, whereas the whole image is around 300000.
  //TODO: see if tranformPoint fast enough, if not you can prefer using a transformation matrix there.
  const uint image_height = ptr_cloud->points.size () / image_width;
  geometry_msgs::PointStamped pt, pt_transformed;
  cv::Mat depth_map = cv::Mat::zeros (image_height, image_width, CV_32FC1);
  for (uint i = 0; i < indices.size (); i++)
  {
    int row_id = indices[i] / image_width;
    int col_id = indices[i] % image_width;

    pt.header = ptr_cloud->header;
    pt.header.stamp = ros::Time (0);
    pt.point.x = ptr_cloud->points[indices[i]].x;
    pt.point.y = ptr_cloud->points[indices[i]].y;
    pt.point.z = ptr_cloud->points[indices[i]].z;

    listener.transformPoint (sensor_frame, pt, pt_transformed);
    depth_map.at<float> (row_id, col_id) = pt_transformed.point.z;
  }
  return depth_map;
}

cv::Mat
al::math::extractDepthMap (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud, const std::vector<int> &indices,
                           const tf::TransformListener &listener, const std::string sensor_frame,
                           const float sampling_ratio)
{
  //DEV_NOTE: it doesn't make sense to transform whole cloud, since we are interested in only a part of it (object).
  //which is usually around 1000points, whereas the whole image is around 300000.
  //TODO: see if tranformPoint fast enough, if not you can prefer using a transformation matrix there.

  uint minus_x_most_id, plus_x_most_id, minus_y_most_id, plus_y_most_id;
  float minus_x_most = FLT_MAX;
  float plus_x_most = FLT_MIN;
  float minus_y_most = FLT_MAX;
  float plus_y_most = FLT_MIN;
  for (uint i = 0; i < indices.size (); i++)
  {
    pcl::PointXYZ p = ptr_cloud->points[indices[i]];
    if (p.x < minus_x_most)
    {
      minus_x_most = p.x;
      minus_x_most_id = indices[i];
    }
    if (p.x > plus_x_most)
    {
      plus_x_most = p.x;
      plus_x_most_id = indices[i];
    }
    if (p.y < minus_y_most)
    {
      minus_y_most = p.y;
      minus_y_most_id = indices[i];
    }
    if (p.y > plus_y_most)
    {
      plus_y_most = p.y;
      plus_y_most_id = indices[i];
    }
  }
  int img_height = ceilf ((plus_x_most - minus_x_most) / sampling_ratio);
  int img_width = ceilf ((plus_y_most - minus_y_most) / sampling_ratio);

  geometry_msgs::PointStamped pt, pt_transformed;
  cv::Mat depth_map = cv::Mat::zeros (img_height, img_width, CV_32FC1);
  for (uint i = 0; i < indices.size (); i++)
  {
    //    int row_id = indices[i] / img_width;
    //    int col_id = indices[i] % img_width;
    pcl::PointXYZ p = ptr_cloud->points[indices[i]];
    int row_id = (p.x - minus_x_most) / sampling_ratio;
    int col_id = (p.y - minus_y_most) / sampling_ratio;

    pt.header = ptr_cloud->header;
    pt.header.stamp = ros::Time (0);
    pt.point.x = p.x;
    pt.point.y = p.y;
    pt.point.z = p.z;

    listener.transformPoint (sensor_frame, pt, pt_transformed);
    depth_map.at<float> (row_id, col_id) = pt_transformed.point.z;
  }
  return depth_map;
}

//assumes circularly symmetric gaussian
cv::Mat
al::math::generateGaussianKernel (const float sigma, const int k_rows, const int k_cols)
{
  int row_shift = k_rows / 2;
  int col_shift = k_cols / 2;
  cv::Mat kernel = cv::Mat::zeros (k_rows, k_cols, CV_32FC1);

  for (int ri = 0; ri < k_rows; ++ri)
  {
    for (int ci = 0; ci < k_cols; ++ci)
    {
      kernel.at<float> (ri, ci) = exp (
          -0.5 * (pow ((ri - row_shift) / sigma, 2.0) + pow ((ci - col_shift) / sigma, 2.0)))
          / (2 * M_PI * sigma * sigma);
    }
  }
  return kernel;
}

//TODO: this should return grid_ also so that it can be accessed later
bool
al::math::downSampleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_downsampled_cloud, float sampling_ratio)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid_;
  grid_.setSaveLeafLayout (true);
  grid_.setLeafSize (sampling_ratio, sampling_ratio, sampling_ratio);
  grid_.setDownsampleAllData (true);
  grid_.setInputCloud (ptr_cloud);
  grid_.filter (*pointcloud_data_downsampled);
  ptr_downsampled_cloud = pointcloud_data_downsampled;
  return true;
}

bool
al::math::downSampleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_downsampled_cloud,
                           pcl::VoxelGrid<pcl::PointXYZ> &grid, float sampling_ratio)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  grid.setSaveLeafLayout (true);
  grid.setLeafSize (sampling_ratio, sampling_ratio, sampling_ratio);
  grid.setDownsampleAllData (true);
  grid.setInputCloud (ptr_cloud);
  grid.filter (*pointcloud_data_downsampled);
  ptr_downsampled_cloud = pointcloud_data_downsampled;
  return true;
}

cv::Mat
al::math::generateKernel (const float a)
{
  cv::Mat kernel = cv::Mat::zeros (5, 1, CV_32FC1);
  cv::Mat kernel_transposed = cv::Mat::zeros (1, 5, CV_32FC1);
  kernel.at<float> (0, 0) = 0.25 - a / 2;
  kernel.at<float> (1, 0) = 0.25;
  kernel.at<float> (2, 0) = a;
  kernel.at<float> (3, 0) = 0.25;
  kernel.at<float> (4, 0) = 0.25 - a / 2;

  cv::transpose (kernel, kernel_transposed);

  return kernel * kernel_transposed;
}

cv::Mat
al::math::expand (const cv::Mat & src, const cv::Mat & generative_kernel, const cv::Size dst_dize)
{
  //if src has a size of (m,n), dst should have (ceil(m/2),ceil(n/2))
  //  cv::Mat dst = cv::Mat::ones (2 * src.rows - 1, 2 * src.cols - 1, CV_32FC1) * MAX_ELEMENT_VAL;
  cv::Mat dst = cv::Mat::ones (dst_dize, CV_32FC1) * MAX_ELEMENT_VAL;

  //padding size
  const int ver_pad_size = generative_kernel.rows / 2;
  const int hor_pad_size = generative_kernel.cols / 2;

  //for normalization of the mask. Most of its entries aren't going to be used in convolution
  //this works at least for the kernel generated from [0.05 0.25 0.4 0.25 0.05].
  //Theoretically it should work for other kernels too.
  cv::Mat kernel = generative_kernel * 4;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < dst.rows; i++)
  {
    for (int j = 0; j < dst.cols; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      //      std::cout << src.at<int> (i / 2, j / 2) << std::endl;
      if (src.at<float> (i / 2, j / 2) == MAX_ELEMENT_VAL)
        continue;

      float sum_mat_tmp = 0;
      float renorm = 0;

      //weighted sum of the src elements around the point (2*i, 2*j) with the kernel
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          float r_index = (i - ri) / 2.0;
          float c_index = (j - ci) / 2.0;

          if (r_index < 0 || r_index >= src.rows || c_index < 0 || c_index >= src.cols)
            continue;

          //consider only integer-indexed entries
          if ((floorf (r_index) == r_index) && (floorf (c_index) == c_index))
          {
            //consider only the actual elements (e.g. ignore non-element entries --MAX_ELEMENT_VAL)
            if (src.at<float> (r_index, c_index) != MAX_ELEMENT_VAL)
            {
              //              std::cout << "i: " << i << "   j: " << j << "   ri: " << ri << "   ci: " << ci << "   r_id: " << r_index
              //                  << "   c_id: " << c_index << "   sum_mat_tmp: " << sum_mat_tmp << "src: "
              //                  << src.at<float> (r_index, c_index) << std::endl;
              sum_mat_tmp += src.at<float> (r_index, c_index) * kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              renorm += kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            }
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
        sum_mat_tmp = sum_mat_tmp / renorm;

      //      if (sum_mat_tmp > 2 || sum_mat_tmp < -2)
      //      {
      //        std::cout << "**" << std::endl;
      //        std::cout << "sum_mat_tmp: " << sum_mat_tmp << std::endl;
      //        std::cout << "i: " << i << "   j: " << j << std::endl;
      //      }

      dst.at<float> (i, j) = sum_mat_tmp;
    }
  }
  return dst;
}

cv::Mat
al::math::expand (const cv::Mat & src, const cv::Mat & generative_kernel)
{
  //if src has a size of (m,n), dst should have (ceil(m/2),ceil(n/2))
  cv::Mat dst = cv::Mat::ones (2 * src.rows - 1, 2 * src.cols - 1, CV_32FC1) * MAX_ELEMENT_VAL;

  //padding size
  const int ver_pad_size = generative_kernel.rows / 2;
  const int hor_pad_size = generative_kernel.cols / 2;

  //for normalization of the mask. Most of its entries aren't going to be used in convolution
  //this works at least for the kernel generated from [0.05 0.25 0.4 0.25 0.05].
  //Theoretically it should work for other kernels too.
  cv::Mat kernel = generative_kernel * 4;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < dst.rows; i++)
  {
    for (int j = 0; j < dst.cols; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      //      std::cout << src.at<int> (i / 2, j / 2) << std::endl;
      if (src.at<float> (i / 2, j / 2) == MAX_ELEMENT_VAL)
        continue;

      float sum_mat_tmp = 0;
      float renorm = 0;

      //weighted sum of the src elements around the point (2*i, 2*j) with the kernel
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          float r_index = (i - ri) / 2.0;
          float c_index = (j - ci) / 2.0;

          if (r_index < 0 || r_index >= src.rows || c_index < 0 || c_index >= src.cols)
            continue;

          //consider only integer-indexed entries
          if ((floorf (r_index) == r_index) && (floorf (c_index) == c_index))
          {
            //consider only the actual elements (e.g. ignore non-element entries --MAX_ELEMENT_VAL)
            if (src.at<float> (r_index, c_index) != MAX_ELEMENT_VAL)
            {
              //              std::cout << "i: " << i << "   j: " << j << "   ri: " << ri << "   ci: " << ci << "   r_index: "
              //                  << r_index << "   c_index: " << c_index << std::endl;
              sum_mat_tmp += src.at<float> (r_index, c_index) * kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              renorm += kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            }
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
        sum_mat_tmp = sum_mat_tmp / renorm;

      dst.at<float> (i, j) = sum_mat_tmp;
    }
  }
  return dst;
}

cv::Mat
al::math::reduce (const cv::Mat & src, const cv::Mat & generative_kernel)
{
  //if src has a size of (m,n), dst should have (ceil(m/2),ceil(n/2))
  cv::Mat dst = cv::Mat::ones (ceilf (src.rows / 2.0), ceilf (src.cols / 2.0),
  CV_32FC1) * MAX_ELEMENT_VAL;

  //padding size
  const int ver_pad_size = generative_kernel.rows / 2;
  const int hor_pad_size = generative_kernel.cols / 2;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < dst.rows; i++)
  {
    for (int j = 0; j < dst.cols; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      if (src.at<float> (2 * i, 2 * j) == MAX_ELEMENT_VAL)
        continue;

      float sum_mat_tmp = 0;
      float renorm = 0;

      //weigted sum of the src elements around the point (2*i, 2*j) with the size of the kernel
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          int r_index = 2 * i + ri;
          int c_index = 2 * j + ci;
          if (r_index < 0 || r_index >= src.rows || c_index < 0 || c_index >= src.cols)
            continue;
          if (src.at<float> (r_index, c_index) != MAX_ELEMENT_VAL)
          {
            //            std::cout << "i: " << i << "   j: " << j << "src: " << src.at<float> (r_index, c_index) << "   ri: " << ri
            //                << "   ci: " << ci << "   r_index: " << r_index << "   c_index: " << c_index << std::endl;
            sum_mat_tmp += src.at<float> (r_index, c_index)
                * generative_kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            renorm += generative_kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
        sum_mat_tmp = sum_mat_tmp / renorm;

      dst.at<float> (i, j) = sum_mat_tmp;
    }
  }
  return dst;
}

std::vector<cv::Mat>
al::math::buildGaussianPyramid (const cv::Mat &src, const cv::Mat & generative_kernel, bool resize_to_original,
                                const uint n_level)
{
  std::vector<cv::Mat> dst_imgs (n_level);

  dst_imgs[0] = src;
  for (uint li = 1; li < n_level; li++)
  {
    //    std::cout << "reducing: " << li << "rows: " << dst_imgs[li - 1].rows << "cols: " << dst_imgs[li - 1].cols << "\n";
    dst_imgs[li] = reduce (dst_imgs[li - 1], generative_kernel);
    //    std::cout << "reduced: " << dst_imgs[li].rows << "\t" << dst_imgs[li].cols << "\n";
  }

  if (resize_to_original)
  {
    for (uint li = 1; li < n_level; li++)
    {
      //expand till expansion index (ei) reaches level index (li) 0
      for (uint ei = li; ei > 0; ei--)
        dst_imgs[li] = expand (dst_imgs[li], generative_kernel,
                               cv::Size (dst_imgs[li - 1].cols, dst_imgs[li - 1].rows));
      //      std::cout << "expanded: " << dst_imgs[li].rows << "\t" << dst_imgs[li].cols << "\n";
    }
  }
  return dst_imgs;
}

std::vector<pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr>
al::math::buildPyramid (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel,
                        bool resize_to_original, const uint n_level, const int n_neighbors)
{
  std::vector<pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr> pyramidded_clouds (n_level);
  for (uint i = 0; i < n_level; i++)
  {
    pyramidded_clouds[i].reset (new pcl::PointCloud<pcl::PointXYZPCurvs>);
    pyramidded_clouds[i]->header.frame_id = ptr_resampled_cloud->header.frame_id;
  }

  pyramidded_clouds[0]->points.resize (ptr_resampled_cloud->points.size ());
  pyramidded_clouds[0]->height = ptr_resampled_cloud->height;
  pyramidded_clouds[0]->width = ptr_resampled_cloud->width;
  for (uint i = 0; i < ptr_resampled_cloud->points.size (); i++)
  {
    pyramidded_clouds[0]->points[i].x = ptr_resampled_cloud->points[i].x;
    pyramidded_clouds[0]->points[i].y = ptr_resampled_cloud->points[i].y;
    pyramidded_clouds[0]->points[i].z = ptr_resampled_cloud->points[i].z;
  }

  for (uint li = 1; li < n_level; li++)
  {
    std::cout << "reducing: " << li << "rows: " << pyramidded_clouds[li - 1]->height << "cols: "
        << pyramidded_clouds[li - 1]->width << "\n";
    pyramidded_clouds[li] = reduce (pyramidded_clouds[li - 1], kernel);
    std::cout << "reduced: " << "rows: " << pyramidded_clouds[li]->height << "cols: " << pyramidded_clouds[li]->width
        << "\n";
  }

  //pointcloud for normal estimation and so curvature estimation should be free of NAN data.
  for (int li = n_level - 1; li >= 0; li--)
  {
    pcl::PointIndices::Ptr valid_indices (new pcl::PointIndices ());
    valid_indices->indices.resize (pyramidded_clouds[li]->points.size ());
    int cnt = 0;
    for (uint i = 0; i < pyramidded_clouds[li]->points.size (); i++)
    {
      if (!isnanf (pyramidded_clouds[li]->points[i].z))
      {
        valid_indices->indices[cnt] = i;
        cnt++;
      }
    }
    valid_indices->indices.resize (cnt);
    //    for (uint i = 0; i < valid_indices->indices.size (); i++)
    //      std::cout << valid_indices->indices[i] << std::endl;
    //extract indices and obtain clear pointcloud, and check if the points are located in the correct order
    pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr valid_cloud (new pcl::PointCloud<pcl::PointXYZPCurvs>);
    pcl::ExtractIndices<pcl::PointXYZPCurvs> extract;
    extract.setInputCloud (pyramidded_clouds[li]);
    extract.setIndices (valid_indices);
    extract.setNegative (false);
    extract.filter (*valid_cloud);
    //    for (uint i = 0; i < valid_cloud->points.size (); i++)
    //      std::cout << valid_cloud->points[i].x << " " << valid_cloud->points[i].y << " " << valid_cloud->points[i].z
    //          << "\n";

    //assume that indices vector is aligned with the way valid_cloud indices are assigned
    //then, we already have our map. valid_indices[0]=100 means 0th point in valid_cloud is 100th point in pyramidded_clouds[li]
    pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZPCurvs>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZPCurvs> ());
    tree->setInputCloud (valid_cloud);
    pcl::NormalEstimationOMP<pcl::PointXYZPCurvs, pcl::Normal> ne_;
    //TODO: viewpoint needs to be an input argument
    ne_.setViewPoint (0, 0, 0); //this could be done in a more informed way by getting this information from a topic
    ne_.setSearchMethod (tree);
    ne_.setKSearch (n_neighbors);
    ne_.setInputCloud (valid_cloud);
    ne_.compute (*pointcloud_normals);

    //    for (uint k = 0; k < pointcloud_normals->points.size (); k++)
    //      std::cout << pointcloud_normals->points[k] << std::endl;

    //TODO: calculate PCurvs here. Assign to the cloud and proceed with the expand
    calcCloudPCurvs (valid_cloud, pointcloud_normals, tree, 15, GOLDFEATHER2004);
    for (uint i = 0; i < valid_indices->indices.size (); i++)
    {
      //      std::cout << valid_cloud->points[i].pc_1 << "\t" << valid_cloud->points[i].pc_2 << std::endl;
      pyramidded_clouds[li]->points[valid_indices->indices[i]].pc_1 = valid_cloud->points[i].pc_1;
      pyramidded_clouds[li]->points[valid_indices->indices[i]].pc_2 = valid_cloud->points[i].pc_2;
    }
  }

  if (resize_to_original)
  {
    for (uint li = n_level - 1; li > 0; --li)
    {
      std::cout << "level: " << (int)li << std::endl;
      //expand till expansion index (ei) reaches level index (li) 0
      for (uint ei = li; ei > 0; --ei)
      {
        std::cout << "expanding: " << pyramidded_clouds[li]->height << "\t" << pyramidded_clouds[li]->width << "\n";
        pyramidded_clouds[li] = expand (
            pyramidded_clouds[li], kernel,
            Eigen::Vector2i (pyramidded_clouds[ei - 1]->height, pyramidded_clouds[ei - 1]->width));
        std::cout << "expanded: " << pyramidded_clouds[li]->height << "\t" << pyramidded_clouds[li]->width << "\n";
      }
      for (uint i = 0; i < pyramidded_clouds[li]->points.size (); i++)
      {
        pyramidded_clouds[li]->points[i].x = ptr_resampled_cloud->points[i].x;
        pyramidded_clouds[li]->points[i].y = ptr_resampled_cloud->points[i].y;
      }
    }
  }

  return pyramidded_clouds;
}

pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr
al::math::reduce (pcl::PointCloud<pcl::PointXYZPCurvs>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel)
{
  pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr ptr_reduced_cloud;
  int height = ceilf (ptr_resampled_cloud->height / 2.0);
  int width = ceilf (ptr_resampled_cloud->width / 2.0);
  ptr_reduced_cloud.reset (new pcl::PointCloud<pcl::PointXYZPCurvs>);
  ptr_reduced_cloud->header.frame_id = ptr_resampled_cloud->header.frame_id;
  ptr_reduced_cloud->points.resize (height * width);
  ptr_reduced_cloud->height = height;
  ptr_reduced_cloud->width = width;
  //if src has a size of (m,n), dst should have (ceil(m/2),ceil(n/2))

  //padding size
  const int ver_pad_size = kernel.rows / 2;
  const int hor_pad_size = kernel.cols / 2;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      int ri_src_cloud = 2 * i;
      int ci_src_cloud = 2 * j;
      int src_cloud_index = ri_src_cloud * ptr_resampled_cloud->width + ci_src_cloud;
      if (isnanf (ptr_resampled_cloud->points[src_cloud_index].z))
        continue;

      float sum_mat_tmp = 0;
      float renorm = 0;

      //weigted sum of the src elements around the point (2*i, 2*j) with the size of the kernel
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          int r_index = ri_src_cloud + ri;
          int c_index = ci_src_cloud + ci;
          if (r_index < 0 || r_index >= (int)ptr_resampled_cloud->height || c_index < 0
              || c_index >= (int)ptr_resampled_cloud->width)
            continue;

          int shifted_src_cloud_index = r_index * ptr_resampled_cloud->width + c_index;
          if (!isnanf (ptr_resampled_cloud->points[shifted_src_cloud_index].z))
          {
            //            std::cout << "i: " << i << "   j: " << j << "src: " << src.at<float> (r_index, c_index) << "   ri: " << ri
            //                << "   ci: " << ci << "   r_index: " << r_index << "   c_index: " << c_index << std::endl;
            sum_mat_tmp += ptr_resampled_cloud->points[shifted_src_cloud_index].z
                * kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            renorm += kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
        sum_mat_tmp = sum_mat_tmp / renorm;

      int dst_cloud_index = i * width + j;
      ptr_reduced_cloud->points[dst_cloud_index].x = ptr_resampled_cloud->points[src_cloud_index].x;
      ptr_reduced_cloud->points[dst_cloud_index].y = ptr_resampled_cloud->points[src_cloud_index].y;
      ptr_reduced_cloud->points[dst_cloud_index].z = sum_mat_tmp;
    }
  }
  return ptr_reduced_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
al::math::reduce (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_reduced_cloud;
  int height = ceilf (ptr_resampled_cloud->height / 2.0);
  int width = ceilf (ptr_resampled_cloud->width / 2.0);
  ptr_reduced_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
  ptr_reduced_cloud->header.frame_id = ptr_resampled_cloud->header.frame_id;
  ptr_reduced_cloud->points.resize (height * width);
  ptr_reduced_cloud->height = height;
  ptr_reduced_cloud->width = width;
  //if src has a size of (m,n), dst should have (ceil(m/2),ceil(n/2))

  //padding size
  const int ver_pad_size = kernel.rows / 2;
  const int hor_pad_size = kernel.cols / 2;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      int ri_src_cloud = 2 * i;
      int ci_src_cloud = 2 * j;
      int src_cloud_index = ri_src_cloud * ptr_resampled_cloud->width + ci_src_cloud;
      if (isnanf (ptr_resampled_cloud->points[src_cloud_index].z)
          || src_cloud_index >= (int)ptr_resampled_cloud->points.size ())
        continue;

      float sum_mat_tmp = 0;
      float renorm = 0;

      //weigted sum of the src elements around the point (2*i, 2*j) with the size of the kernel
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          int r_index = ri_src_cloud + ri;
          int c_index = ci_src_cloud + ci;
          if (r_index < 0 || r_index >= (int)ptr_resampled_cloud->height || c_index < 0
              || c_index >= (int)ptr_resampled_cloud->width)
            continue;

          int shifted_src_cloud_index = r_index * ptr_resampled_cloud->width + c_index;
          if (!isnanf (ptr_resampled_cloud->points[shifted_src_cloud_index].z))
          {
            //            std::cout << "i: " << i << "   j: " << j << "src: " << src.at<float> (r_index, c_index) << "   ri: " << ri
            //                << "   ci: " << ci << "   r_index: " << r_index << "   c_index: " << c_index << std::endl;
            sum_mat_tmp += ptr_resampled_cloud->points[shifted_src_cloud_index].z
                * kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            renorm += kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
        sum_mat_tmp = sum_mat_tmp / renorm;

      int dst_cloud_index = i * width + j;
      ptr_reduced_cloud->points[dst_cloud_index].x = ptr_resampled_cloud->points[src_cloud_index].x;
      ptr_reduced_cloud->points[dst_cloud_index].y = ptr_resampled_cloud->points[src_cloud_index].y;
      ptr_reduced_cloud->points[dst_cloud_index].z = sum_mat_tmp;
    }
  }
  return ptr_reduced_cloud;
}

pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr
al::math::expand (pcl::PointCloud<pcl::PointXYZPCurvs>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel,
                  const Eigen::Vector2i expansion_size)
{
  pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr ptr_expanded_cloud;
  //  int height = ceilf (2 * ptr_resampled_cloud->height - 1);
  //  int width = ceilf (2 * ptr_resampled_cloud->width - 1);
  int height = expansion_size[0];
  int width = expansion_size[1];
  ptr_expanded_cloud.reset (new pcl::PointCloud<pcl::PointXYZPCurvs>);
  ptr_expanded_cloud->points.resize (height * width);

  ptr_expanded_cloud->height = height;
  ptr_expanded_cloud->width = width;
  ptr_expanded_cloud->header.frame_id = ptr_resampled_cloud->header.frame_id;

  //padding size
  const int ver_pad_size = kernel.rows / 2;
  const int hor_pad_size = kernel.cols / 2;

  //for normalization of the mask. Most of its entries aren't going to be used in convolution
  //this works for the kernels generated from [0.25-a/2 0.25 a 0.25 0.25-a/2].
  cv::Mat kernel_ = kernel * 4;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      int ri_src_cloud = i / 2;
      int ci_src_cloud = j / 2;
      int src_cloud_index = ri_src_cloud * ptr_resampled_cloud->width + ci_src_cloud;
      if (isnanf (ptr_resampled_cloud->points[src_cloud_index].z))
        continue;

      float sum_depth_tmp = 0;
      float sum_pc_1_tmp = 0;
      float sum_pc_2_tmp = 0;
      float renorm = 0;

      //weighted sum of the src elements around the point (i/2, j/2) with the kernel_
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          float r_index = (i - ri) / 2.0;
          float c_index = (j - ci) / 2.0;
          int shifted_src_cloud_index = r_index * ptr_resampled_cloud->width + c_index;
          if (r_index < 0 || r_index >= ptr_resampled_cloud->height || c_index < 0
              || c_index >= ptr_resampled_cloud->width)
            continue;

          //consider only integer-indexed entries
          if ((floorf (r_index) == r_index) && (floorf (c_index) == c_index))
          {
            //consider only the actual elements (e.g. ignore non-element entries --e.g. NANFs)
            if (!isnanf (ptr_resampled_cloud->points[shifted_src_cloud_index].z))
            {
              //              std::cout << "i: " << i << "   j: " << j << "   ri: " << ri << "   ci: " << ci << "   r_index: "
              //                  << r_index << "   c_index: " << c_index << std::endl;
              sum_depth_tmp += ptr_resampled_cloud->points[shifted_src_cloud_index].z
                  * kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              sum_pc_1_tmp += ptr_resampled_cloud->points[shifted_src_cloud_index].pc_1
                  * kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              sum_pc_2_tmp += ptr_resampled_cloud->points[shifted_src_cloud_index].pc_2
                  * kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              renorm += kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            }
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
      {
        sum_depth_tmp = sum_depth_tmp / renorm;
        sum_pc_1_tmp = sum_pc_1_tmp / renorm;
        sum_pc_2_tmp = sum_pc_2_tmp / renorm;
      }

      int dst_cloud_index = i * width + j;
      ptr_expanded_cloud->points[dst_cloud_index].z = sum_depth_tmp;
      ptr_expanded_cloud->points[dst_cloud_index].pc_1 = sum_pc_1_tmp;
      ptr_expanded_cloud->points[dst_cloud_index].pc_2 = sum_pc_2_tmp;
    }
  }
  return ptr_expanded_cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr>
al::math::extractScaledLabeledCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_raw_cloud, int n_pyr_levels,
                                     cv::Mat kernel, float sampling_ratio, int n_neighbors, int curv_est_method)
{
  std::vector<pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr> scaled_labeled_clouds (n_pyr_levels);
  for (int i = 0; i < n_pyr_levels; i++)
  {
    scaled_labeled_clouds[i].reset (new pcl::PointCloud<pcl::PointXYZCurvatures>);
    scaled_labeled_clouds[i]->header.frame_id = ptr_raw_cloud->header.frame_id;
  }

  //re-sample raw_cloud to the given sampling ratio and
  //make it rectangle-framed by filling invalid entries with NAN points. This will help pyramiding procedure
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_resampled_cloud;
  al::math::resampleCloud (ptr_raw_cloud, ptr_resampled_cloud, sampling_ratio);

  //transfer necessary information to the 0th level cloud.
  scaled_labeled_clouds[0]->points.resize (ptr_resampled_cloud->points.size ());
  scaled_labeled_clouds[0]->height = ptr_resampled_cloud->height;
  scaled_labeled_clouds[0]->width = ptr_resampled_cloud->width;
  for (uint i = 0; i < ptr_resampled_cloud->points.size (); i++)
  {
    scaled_labeled_clouds[0]->points[i].x = ptr_resampled_cloud->points[i].x;
    scaled_labeled_clouds[0]->points[i].y = ptr_resampled_cloud->points[i].y;
    scaled_labeled_clouds[0]->points[i].z = ptr_resampled_cloud->points[i].z;
  }

  //reduce ptr_resampled_cloud to obtain higher scale versions of it
  pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr tmp;
  for (int li = 1; li < n_pyr_levels; li++)
  {
    std::cout << "reducing: " << li << "rows: " << scaled_labeled_clouds[li - 1]->height << "cols: "
        << scaled_labeled_clouds[li - 1]->width << "\n";
    std::cout << "a" << std::endl;
    reduce (scaled_labeled_clouds[li - 1], tmp, kernel);
    std::cout << "b" << std::endl;
    std::cout << "reduced: " << "rows: " << tmp->height << "cols: " << tmp->width << "\n";
    scaled_labeled_clouds[li] = tmp;
    std::cout << "c" << std::endl;
    std::cout << "reduced: " << "rows: " << scaled_labeled_clouds[li]->height << "cols: "
        << scaled_labeled_clouds[li]->width << "\n";
    std::cout << "d" << std::endl;
  }

  //calculate principal curvatures at each scale. This requires normal estimation, so that points
  //can be mapped to the local coordinates of their own. But, we need to clean the clouds from NANs
  //before further proceeding.
  pcl::ExtractIndices<pcl::PointXYZCurvatures> extract;
  pcl::PointIndices::Ptr valid_indices (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr valid_cloud (new pcl::PointCloud<pcl::PointXYZCurvatures>);
  pcl::search::KdTree<pcl::PointXYZCurvatures>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZCurvatures> ());
  pcl::NormalEstimationOMP<pcl::PointXYZCurvatures, pcl::Normal> ne_;
  pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals (new pcl::PointCloud<pcl::Normal>);
  for (int li = n_pyr_levels - 1; li >= 0; li--)
  {
    valid_indices->indices.resize (scaled_labeled_clouds[li]->points.size ());
    int cnt = 0;
    for (uint i = 0; i < scaled_labeled_clouds[li]->points.size (); i++)
    {
      if (!isnanf (scaled_labeled_clouds[li]->points[i].z))
      {
        valid_indices->indices[cnt] = i;
        cnt++;
      }
    }
    valid_indices->indices.resize (cnt);
    extract.setInputCloud (scaled_labeled_clouds[li]);
    extract.setIndices (valid_indices);
    extract.setNegative (false);
    extract.filter (*valid_cloud);

    tree->setInputCloud (valid_cloud);
    ne_.setViewPoint (0, 0, 0); //clouds are in the frame of sensor, so the viewpoint is origin.
    ne_.setSearchMethod (tree);
    ne_.setKSearch (n_neighbors);
    ne_.setInputCloud (valid_cloud);
    ne_.compute (*pointcloud_normals);

    al::math::calcCloudPCurvatures (valid_cloud, pointcloud_normals, tree, n_neighbors, curv_est_method);
    for (uint i = 0; i < valid_indices->indices.size (); i++)
    {
      scaled_labeled_clouds[li]->points[valid_indices->indices[i]].pc1 = valid_cloud->points[i].pc1;
      scaled_labeled_clouds[li]->points[valid_indices->indices[i]].pc2 = valid_cloud->points[i].pc2;
    }
  }

  //expand reduced clouds by processing their depth, pc1 and pc2 fields
  for (uint li = n_pyr_levels - 1; li > 0; --li)
  {
    std::cout << "level: " << (int)li << std::endl;
    //expand till expansion index (ei) reaches level index (li) 0
    for (uint ei = li; ei > 0; --ei)
    {
      std::cout << "expanding: " << scaled_labeled_clouds[li]->height << "\t" << scaled_labeled_clouds[li]->width
          << "\n";
      expand (scaled_labeled_clouds[li], tmp, kernel,
              Eigen::Vector2i (scaled_labeled_clouds[ei - 1]->height, scaled_labeled_clouds[ei - 1]->width));
      scaled_labeled_clouds[li] = tmp;
      std::cout << "expanded: " << scaled_labeled_clouds[li]->height << "\t" << scaled_labeled_clouds[li]->width
          << "\n";
    }
    for (uint i = 0; i < scaled_labeled_clouds[li]->points.size (); i++)
    {
      scaled_labeled_clouds[li]->points[i].x = ptr_resampled_cloud->points[i].x;
      scaled_labeled_clouds[li]->points[i].y = ptr_resampled_cloud->points[i].y;
    }
  }

  //calculate hksc values from principal curvature values
  //and //label calculated hksc values
  for (uint li = 0; li < scaled_labeled_clouds.size (); li++)
  {
    al::math::calcCloudHKSCFromPCurvatures (scaled_labeled_clouds[li]);
    al::math::labelCloudCurvatures (scaled_labeled_clouds[li], HK);
    al::math::labelCloudCurvatures (scaled_labeled_clouds[li], SC);
  }

  return scaled_labeled_clouds;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
al::math::resampleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_resampled_cloud, float sampling_ratio)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_raw_resampled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ptr_raw_resampled_cloud->header.frame_id = ptr_cloud->header.frame_id;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setSaveLeafLayout (true);
  grid.setLeafSize (sampling_ratio, sampling_ratio, sampling_ratio);
  grid.setDownsampleAllData (true);
  grid.setInputCloud (ptr_cloud);
  grid.filter (*ptr_raw_resampled_cloud);

  ROS_INFO("raw_resampled_cloud has %d points", (int )ptr_raw_resampled_cloud->points.size ());

  ptr_resampled_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
  ptr_resampled_cloud->header.frame_id = ptr_cloud->header.frame_id;
  ptr_resampled_cloud->points.resize (grid.getNrDivisions ()[0] * grid.getNrDivisions ()[1]);
  const Eigen::Vector3i v_ref = grid.getMinBoxCoordinates ();
  for (int i = 0; i < grid.getNrDivisions ()[0]; i++)
  {
    for (int j = 0; j < grid.getNrDivisions ()[1]; j++)
    {
      Eigen::Vector3f p (0, 0, 0);
      int cnt_same_z = 0;
      for (int k = 0; k < grid.getNrDivisions ()[2]; k++)
      {
        Eigen::Vector3i v (i, j, k);
        v = v + v_ref;
        int index = grid.getCentroidIndexAt (v);

        if (index != -1)
        {
          Eigen::Vector3f grid_leaf_size = grid.getLeafSize ();
          p = p + Eigen::Vector3f (v[0] * grid_leaf_size[0], v[1] * grid_leaf_size[1], v[2] * grid_leaf_size[2]);

          cnt_same_z++;
        }
      }
      if (cnt_same_z)
      {
        p /= cnt_same_z;
        pcl::PointXYZ point (p[0], p[1], p[2]);
        ptr_resampled_cloud->points[i * grid.getNrDivisions ()[1] + j] = point;
      }
      else
      {
        pcl::PointXYZ point (p[0], p[1], NAN);
        ptr_resampled_cloud->points[i * grid.getNrDivisions ()[1] + j] = point;
      }
    }
  }
  ptr_resampled_cloud->height = grid.getNrDivisions ()[0];
  ptr_resampled_cloud->width = grid.getNrDivisions ()[1];

  ROS_INFO("resampled_cloud has %d points", (int )ptr_resampled_cloud->points.size ());

  return ptr_raw_resampled_cloud;
}

bool
al::math::reduce (pcl::PointCloud<pcl::PointXYZCurvatures>::ConstPtr ptr_src_cloud,
                  pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &ptr_dst_cloud, const cv::Mat & kernel)
{
  //if src has a size of (m,n), dst should have (ceil(m/2),ceil(n/2))
  int height = ceilf (ptr_src_cloud->height / 2.0);
  int width = ceilf (ptr_src_cloud->width / 2.0);
  ptr_dst_cloud.reset (new pcl::PointCloud<pcl::PointXYZCurvatures>);
  ptr_dst_cloud->header.frame_id = ptr_src_cloud->header.frame_id;
  ptr_dst_cloud->points.resize (height * width);
  ptr_dst_cloud->height = height;
  ptr_dst_cloud->width = width;

  //padding size
  const int ver_pad_size = kernel.rows / 2;
  const int hor_pad_size = kernel.cols / 2;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      int ri_src_cloud = 2 * i;
      int ci_src_cloud = 2 * j;
      int src_cloud_index = ri_src_cloud * ptr_src_cloud->width + ci_src_cloud;
      if (isnanf (ptr_src_cloud->points[src_cloud_index].z) || src_cloud_index >= (int)ptr_src_cloud->points.size ())
        continue;

      float tmp_convolution = 0;
      float renorm = 0;

      //weigted sum of the src elements around the point (2*i, 2*j) with the size of the kernel
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          int r_index = ri_src_cloud + ri;
          int c_index = ci_src_cloud + ci;
          if (r_index < 0 || r_index >= (int)ptr_src_cloud->height || c_index < 0
              || c_index >= (int)ptr_src_cloud->width)
            continue;

          int shifted_src_cloud_index = r_index * ptr_src_cloud->width + c_index;
          if (!isnanf (ptr_src_cloud->points[shifted_src_cloud_index].z))
          {
            //            std::cout << "i: " << i << "   j: " << j << "src: " << src.at<float> (r_index, c_index) << "   ri: " << ri
            //                << "   ci: " << ci << "   r_index: " << r_index << "   c_index: " << c_index << std::endl;
            tmp_convolution += ptr_src_cloud->points[shifted_src_cloud_index].z
                * kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            renorm += kernel.at<float> (ri + ver_pad_size, ci + hor_pad_size);
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
        tmp_convolution = tmp_convolution / renorm;

      int dst_cloud_index = i * width + j;
      ptr_dst_cloud->points[dst_cloud_index].x = ptr_src_cloud->points[src_cloud_index].x;
      ptr_dst_cloud->points[dst_cloud_index].y = ptr_src_cloud->points[src_cloud_index].y;
      ptr_dst_cloud->points[dst_cloud_index].z = tmp_convolution;
    }
  }
  return true;
}

bool
al::math::calcCloudPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr pointcloud_data,
                                pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                                pcl::search::KdTree<pcl::PointXYZCurvatures>::Ptr tree, const int n_neighbors,
                                const int curv_estimation_method)
{
  for (uint i = 0; i < pointcloud_data->points.size (); i++)
  {
    if (!calcPointPCurvatures (pointcloud_data, pointcloud_normals, tree, i, curv_estimation_method, n_neighbors))
      return false;
  }
  return true;
}

bool
al::math::calcPointPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr pointcloud_data,
                                pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                                pcl::search::KdTree<pcl::PointXYZCurvatures>::Ptr tree, const int pt_idx,
                                const int curv_estimation_method, const int n_neighbors)
{

  std::vector<int> neighbor_ids (n_neighbors);
  std::vector<float> pointsSquaredDist (n_neighbors);
  tree->nearestKSearch (pt_idx, n_neighbors, neighbor_ids, pointsSquaredDist);

  if (curv_estimation_method == GOLDFEATHER2004)
  {
    std::map<int, pcl::PointXYZCurvatures> map_id_to_pt;
    for (uint i = 1; i < neighbor_ids.size (); i++)
      map_id_to_pt[neighbor_ids[i]] = pointcloud_data->points[neighbor_ids[i]];

    //transform neighbor points to local (e.g. the coordinate system located at cloud[pt_idx])
    btMatrix3x3 transformation;
    btVector3 n_i;
    transformation.setIdentity ();
    pcl::Normal normal = pointcloud_normals->points[pt_idx];
    n_i.setValue (normal.normal_x, normal.normal_y, normal.normal_z);
    btVector3 z (0, 0, 1);

    float angle = al::math::getAngleBetween (n_i, z);
    btVector3 axis = n_i.cross (z);

    if (fabs (angle) > 0.05)
      transformation.setRotation (btQuaternion (axis, angle));
    else
      transformation.setIdentity ();
    transformation.setRotation (btQuaternion (axis, angle));

    btVector3 p_i (pointcloud_data->points[pt_idx].x, pointcloud_data->points[pt_idx].y,
                   pointcloud_data->points[pt_idx].z);

    for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
    {
      btVector3 p_n (pointcloud_data->points[neighbor_ids[n]].x, pointcloud_data->points[neighbor_ids[n]].y,
                     pointcloud_data->points[neighbor_ids[n]].z);
      //first translate to the origin
      p_n -= p_i;
      //now rotate
      p_n = transformation * p_n;
      map_id_to_pt[neighbor_ids[n]].x = p_n.x ();
      map_id_to_pt[neighbor_ids[n]].y = p_n.y ();
      map_id_to_pt[neighbor_ids[n]].z = p_n.z ();
    }

    //NN=8, 9 including itself
    //utilizing surface normal information
    std::vector<double> point_based_eqn (7, 0.0);
    std::vector<double> norm_x_based_eqn (7, 0.0);
    std::vector<double> norm_y_based_eqn (7, 0.0);

    Eigen::MatrixXd M ((neighbor_ids.size () - 1) * 3, 7);
    Eigen::MatrixXd R ((neighbor_ids.size () - 1) * 3, 1);
    Eigen::MatrixXd U (7, 1);
    Eigen::MatrixXd W (2, 2);

    int offset_1 = neighbor_ids.size () - 1;
    int offset_2 = 2 * offset_1;

    for (uint n = 1; n < (uint)neighbor_ids.size (); n++)
    {
      btVector3 p_n (map_id_to_pt[neighbor_ids[n]].x, map_id_to_pt[neighbor_ids[n]].y, map_id_to_pt[neighbor_ids[n]].z);
      btVector3 n_n (pointcloud_normals->points[neighbor_ids[n]].normal_x + pointcloud_data->points[neighbor_ids[n]].x,
                     pointcloud_normals->points[neighbor_ids[n]].normal_y + pointcloud_data->points[neighbor_ids[n]].y,
                     pointcloud_normals->points[neighbor_ids[n]].normal_z + pointcloud_data->points[neighbor_ids[n]].z);

      n_n = n_n - p_i;

      n_n = transformation * n_n - p_n;
      M (n - 1, 0) = p_n.x () * p_n.x () * 0.5;
      M (n - 1, 1) = p_n.x () * p_n.y ();
      M (n - 1, 2) = p_n.y () * p_n.y () * 0.5;
      M (n - 1, 3) = p_n.x () * p_n.x () * p_n.x ();
      M (n - 1, 4) = p_n.x () * p_n.x () * p_n.y ();
      M (n - 1, 5) = p_n.x () * p_n.y () * p_n.y ();
      M (n - 1, 6) = p_n.y () * p_n.y () * p_n.y ();

      M (offset_1 + n - 1, 0) = p_n.x ();
      M (offset_1 + n - 1, 1) = p_n.y ();
      M (offset_1 + n - 1, 2) = 0.0;
      M (offset_1 + n - 1, 3) = p_n.x () * p_n.x () * 3;
      M (offset_1 + n - 1, 4) = p_n.x () * p_n.y () * 2;
      M (offset_1 + n - 1, 5) = p_n.y () * p_n.y ();
      M (offset_1 + n - 1, 6) = 0.0;

      M (offset_2 + n - 1, 0) = 0.0;
      M (offset_2 + n - 1, 1) = p_n.x ();
      M (offset_2 + n - 1, 2) = p_n.y ();
      M (offset_2 + n - 1, 3) = 0.0;
      M (offset_2 + n - 1, 4) = p_n.x () * p_n.x ();
      M (offset_2 + n - 1, 5) = 2 * p_n.x () * p_n.y ();
      M (offset_2 + n - 1, 6) = p_n.y () * p_n.y () * 3;

      R (n - 1, 0) = p_n.z ();
      R (offset_1 + n - 1, 0) = -n_n.x () / n_n.z ();
      R (offset_2 + n - 1, 0) = -n_n.y () / n_n.z ();
    }

    //apply least squares
    U = ((M.transpose () * M).inverse () * M.transpose ()) * R;

    W (0, 0) = U (0, 0); //a
    W (0, 1) = U (1, 0); //b
    W (1, 0) = U (1, 0); //b
    W (1, 1) = U (2, 0); //c

    if (W.eigenvalues ().data ()[0].real () > W.eigenvalues ().data ()[1].real ())
    {
      pointcloud_data->points[pt_idx].pc1 = W.eigenvalues ().data ()[0].real ();
      pointcloud_data->points[pt_idx].pc2 = W.eigenvalues ().data ()[1].real ();
    }
    else
    {
      pointcloud_data->points[pt_idx].pc1 = W.eigenvalues ().data ()[1].real ();
      pointcloud_data->points[pt_idx].pc2 = W.eigenvalues ().data ()[0].real ();
    }
  }
  else if (curv_estimation_method == QUADRIC_FITTING)
  {
    const std::vector<double> mat_eqn_coeffs = fitQuadricSurface (pointcloud_data, pt_idx, neighbor_ids,
                                                                  pointcloud_normals);
    tf::Vector3 point (0, 0, 0);
    cv::Mat hessian;
    double f_x, f_y;

    cv::Mat fnc = cv::Mat::zeros (3, 3, CV_64FC1);
    for (uint j = 0; j < mat_eqn_coeffs.size (); j++)
      fnc.at<double> (j / 3, j % 3) = mat_eqn_coeffs[j];

    hessian = al::math::calcHessian (fnc, point, f_x, f_y);

    double h_u = f_x, h_v = f_y;
    double h_uu = hessian.at<double> (0, 0);
    double h_vv = hessian.at<double> (1, 1);
    double h_uv = hessian.at<double> (0, 1);

    double E = 1 + h_u * h_u;
    double F = h_u * h_v;
    double G = 1 + h_v * h_v;

    double h_u_squared = h_u * h_u;
    double h_v_squared = h_v * h_v;
    double denominator = sqrt (1 + h_u_squared + h_v_squared);

    double e = h_uu / denominator;
    double f = h_uv / denominator;
    double g = h_vv / denominator;

    double mean_curv = (e * G - 2 * f * F + g * E) / (2 * (E * G - F * F));
    double gaus_curv = (e * g - f * f) / (E * G - F * F);

    //k_1 is always larger than k_2
    pointcloud_data->points[pt_idx].pc1 = mean_curv + sqrt (mean_curv * mean_curv - gaus_curv);
    pointcloud_data->points[pt_idx].pc2 = mean_curv - sqrt (mean_curv * mean_curv - gaus_curv);
  }
  else
    return false;

  //  std::cout << pointcloud_data->points[pt_idx].pc_1 << "\t" << pointcloud_data->points[pt_idx].pc_2 << std::endl;
  return true;
}

std::vector<double>
al::math::fitQuadricSurface (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr pointcloud_data, const int pt_idx,
                             const std::vector<int> &neighbor_ids, pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals)
{
  std::map<int, pcl::PointXYZCurvatures> map_id_to_pt;
  for (uint i = 1; i < neighbor_ids.size (); i++)
    map_id_to_pt[neighbor_ids[i]] = pointcloud_data->points[neighbor_ids[i]];

  btMatrix3x3 transformation;
  btVector3 n_i;
  transformation.setIdentity ();
  pcl::Normal normal = pointcloud_normals->points[pt_idx];
  n_i.setValue (normal.normal_x, normal.normal_y, normal.normal_z);
  btVector3 z (0, 0, 1);
  float angle = al::math::getAngleBetween (n_i, z);
  btVector3 axis = n_i.cross (z);

  if (fabs (angle) > 0.05)
    transformation.setRotation (btQuaternion (axis, angle));
  else
    transformation.setIdentity ();

  btVector3 p_i (pointcloud_data->points[pt_idx].x, pointcloud_data->points[pt_idx].y,
                 pointcloud_data->points[pt_idx].z);

  for (uint n = 0; n < (uint)neighbor_ids.size (); n++)
  {
    btVector3 p_n (pointcloud_data->points[neighbor_ids[n]].x, pointcloud_data->points[neighbor_ids[n]].y,
                   pointcloud_data->points[neighbor_ids[n]].z);
    //first translate to the origin
    p_n -= p_i;
    //now rotate
    p_n = transformation * p_n;
    map_id_to_pt[neighbor_ids[n]].x = p_n.x ();
    map_id_to_pt[neighbor_ids[n]].y = p_n.y ();
    map_id_to_pt[neighbor_ids[n]].z = p_n.z ();
  }

  int x_row_pow = 0, x_col_pow = 0, y_row_pow = 0, y_col_pow = 0;
  Eigen::Matrix<double, 9, 9> D, D_inv;
  D.setZero (9, 9);
  Eigen::Matrix<double, 9, 1> A, Z;
  Z.setZero (9, 1);
  pcl::PointXYZCurvatures p_n;

  for (uint n = 0; n < (uint)neighbor_ids.size (); n++)
  {
    p_n = map_id_to_pt[neighbor_ids[n]];
    for (uint i = 0; i < 9; i++)
    {
      x_row_pow = i / 3;
      y_row_pow = i % 3;

      Z (i, 0) = Z (i, 0) + p_n.z * pow (p_n.x, x_row_pow) * pow (p_n.y, y_row_pow);

      for (uint j = 0; j < 9; j++)
      {
        x_col_pow = j / 3;
        y_col_pow = j % 3;
        D (i, j) = D (i, j) + pow (p_n.x, x_row_pow + x_col_pow) * pow (p_n.y, y_row_pow + y_col_pow);
      }
    }
  }
  D_inv = D.inverse ();
  A = D_inv * Z;

  std::vector<double> quadric_surface_params (9);
  for (uint i = 0; i < 9; i++)
    quadric_surface_params[i] = A (i, 0);

  return quadric_surface_params;
}

bool
al::math::expand (pcl::PointCloud<pcl::PointXYZCurvatures>::ConstPtr ptr_src_cloud,
                  pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &ptr_dst_cloud, const cv::Mat & kernel,
                  const Eigen::Vector2i expansion_size)
{
  int height = expansion_size[0]; //in x, that is height
  int width = expansion_size[1]; //in y, that is width
  ptr_dst_cloud.reset (new pcl::PointCloud<pcl::PointXYZCurvatures>);
  ptr_dst_cloud->points.resize (height * width);

  ptr_dst_cloud->height = height;
  ptr_dst_cloud->width = width;
  ptr_dst_cloud->header.frame_id = ptr_src_cloud->header.frame_id;

  //padding size
  const int ver_pad_size = kernel.rows / 2;
  const int hor_pad_size = kernel.cols / 2;

  //for normalization of the mask. Most of its entries aren't going to be used in convolution
  //this works for the kernels generated from [0.25-a/2 0.25 a 0.25 0.25-a/2].
  cv::Mat kernel_ = kernel * 4;

  //modified convolution operation while resizing at the same time
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      //if this is a non-element entry, skip it, src doesn't need to be convoluted here
      int ri_src_cloud = i / 2;
      int ci_src_cloud = j / 2;
      int src_cloud_index = ri_src_cloud * ptr_src_cloud->width + ci_src_cloud;
      if (isnanf (ptr_src_cloud->points[src_cloud_index].z))
        continue;

      float sum_depth_tmp = 0;
      float sum_pc_1_tmp = 0;
      float sum_pc_2_tmp = 0;
      float renorm = 0;

      //weighted sum of the src elements around the point (i/2, j/2) with the kernel_
      for (int ri = -ver_pad_size; ri <= ver_pad_size; ri++)
      {
        for (int ci = -hor_pad_size; ci <= hor_pad_size; ci++)
        {
          float r_index = (i - ri) / 2.0;
          float c_index = (j - ci) / 2.0;
          int shifted_src_cloud_index = r_index * ptr_src_cloud->width + c_index;
          if (r_index < 0 || r_index >= ptr_src_cloud->height || c_index < 0 || c_index >= ptr_src_cloud->width)
            continue;

          //consider only integer-indexed entries
          if ((floorf (r_index) == r_index) && (floorf (c_index) == c_index))
          {
            //consider only the actual elements (e.g. ignore non-element entries --e.g. NANFs)
            if (!isnanf (ptr_src_cloud->points[shifted_src_cloud_index].z))
            {
              //              std::cout << "i: " << i << "   j: " << j << "   ri: " << ri << "   ci: " << ci << "   r_index: "
              //                  << r_index << "   c_index: " << c_index << std::endl;
              sum_depth_tmp += ptr_src_cloud->points[shifted_src_cloud_index].z
                  * kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              sum_pc_1_tmp += ptr_src_cloud->points[shifted_src_cloud_index].pc1
                  * kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              sum_pc_2_tmp += ptr_src_cloud->points[shifted_src_cloud_index].pc2
                  * kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
              renorm += kernel_.at<float> (ri + ver_pad_size, ci + hor_pad_size);
            }
          }
        }
      }

      //recover normalization factor which changed due to the existence non-element entries
      if (renorm)
      {
        sum_depth_tmp = sum_depth_tmp / renorm;
        sum_pc_1_tmp = sum_pc_1_tmp / renorm;
        sum_pc_2_tmp = sum_pc_2_tmp / renorm;
      }

      int dst_cloud_index = i * width + j;
      ptr_dst_cloud->points[dst_cloud_index].z = sum_depth_tmp;
      ptr_dst_cloud->points[dst_cloud_index].pc1 = sum_pc_1_tmp;
      ptr_dst_cloud->points[dst_cloud_index].pc2 = sum_pc_2_tmp;
    }
  }
  return true;
}

bool
al::math::calcPointHKSCFromPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data,
                                        const int pt_idx)
{

  float pc1 = pointcloud_data->points[pt_idx].pc1;
  float pc2 = pointcloud_data->points[pt_idx].pc2;
  pointcloud_data->points[pt_idx].h = (pc1 + pc2) / 2.0;
  pointcloud_data->points[pt_idx].k = pc1 * pc2;
  pointcloud_data->points[pt_idx].s = sqrt ((pc1 * pc1 + pc2 * pc2) / 2);

  if (pc1 != pc2)
    pointcloud_data->points[pt_idx].s = atan ((pc1 + pc2) / (pc2 - pc1)) * 2 / M_PI;
  else
  {
    if (pc1 == 0 && pc2 == 0)
      pointcloud_data->points[pt_idx].s = NAN;
    else
      pointcloud_data->points[pt_idx].s = 0;
  }
  return true;
}

bool
al::math::calcCloudHKSCFromPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data)
{
  for (uint i = 0; i < pointcloud_data->points.size (); i++)
    al::math::calcPointHKSCFromPCurvatures (pointcloud_data, i);

  return true;
}

bool
al::math::labelPointCurvature (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data, const int pt_idx,
                               int label_space, float sampling_ratio)
{
  const double c_zero = C_ZERO * (0.0005 / sampling_ratio);
  const double h_zero = c_zero;
  const double k_zero = h_zero * h_zero;

  double h = pointcloud_data->points[pt_idx].h;
  double k = pointcloud_data->points[pt_idx].k;
  double s = pointcloud_data->points[pt_idx].s;
  double c = pointcloud_data->points[pt_idx].c;

  int color_id = 0;
  //TODO: to be implemented later
  if (label_space == HKSC)
  {
    //    if (c > c_zero)
    //    {
    //      if (h < -h_zero)
    //      {
    //        if (k >= k_zero && s >= 5.0 / 8 && s <= 1) //BLUE
    //        {
    //          color.r = 0.0;
    //          color.g = 0.0;
    //          color.b = 1.0;
    //        }
    //        else if (fabs (k) < k_zero && s >= 3.0 / 8 && s <= 5.0 / 8) //MAGENTA
    //        {
    //          color.r = 1.0;
    //          color.g = 0.0;
    //          color.b = 1.0;
    //        }
    //        else if (k < -k_zero && s >= 3.0 / 16 && s <= 3.0 / 8) //RED
    //        {
    //          color.r = 1.0;
    //          color.g = 0.0;
    //          color.b = 0.0;
    //        }
    //      }
    //      else if (h < h_zero)
    //      {
    //        if (k < -k_zero && s >= -3.0 / 16 && s <= 3.0 / 16) //ORANGE
    //        {
    //          color.r = 1.0;
    //          color.g = 0.5;
    //          color.b = 0.0;
    //        }
    //      }
    //      else if (h >= h_zero)
    //      {
    //        if (k >= k_zero && s >= -1 && s <= -5.0 / 8) //CYAN
    //        {
    //          color.r = 0.0;
    //          color.g = 1.0;
    //          color.b = 1.0;
    //        }
    //        else if (fabs (k) < k_zero && s >= -5.0 / 8 && s <= -3.0 / 8) //GREEN
    //        {
    //          color.r = 0.0;
    //          color.g = 1.0;
    //          color.b = 0.0;
    //        }
    //        else if (k < -k_zero && s >= -3.0 / 16 && s <= 3.0 / 16) //YELLOW
    //        {
    //          color.r = 1.0;
    //          color.g = 1.0;
    //          color.b = 0.0;
    //        }
    //      }
    //    }
    //    else //PLANAR
    //    {
    //      if (h < h_zero && k < k_zero) //GREY
    //      {
    //        color.r = 0.5;
    //        color.g = 0.5;
    //        color.b = 0.5;
    //      }
    //    }
  }
  else if (label_space == HK)
  {
    if (h < -h_zero)
    {
      if (k > k_zero)
        color_id = al::viz::BLUE;
      else if (fabs (k) <= k_zero)
        color_id = al::viz::MAGENTA;
      else
        color_id = al::viz::RED;
    }
    else if (fabs (h) <= h_zero)
    {
      if (k > k_zero)
      {
        std::cerr << "impossible curvature calculation!\n";
        return false;
      }
      else if (fabs (k) <= k_zero)
        color_id = al::viz::GREY;
      else
        color_id = al::viz::ORANGE;
    }
    else
    {
      if (k > k_zero)
        color_id = al::viz::CYAN;
      else if (fabs (k) <= k_zero)
        color_id = al::viz::GREEN;
      else
        color_id = al::viz::YELLOW;
    }
  }
  else if (label_space == SC)
  {
    if (c > c_zero)
    {
      if (s >= 5.0 / 8 && s <= 1) //BLUE
        color_id = al::viz::BLUE;
      else if (s >= 3.0 / 8 && s < 5.0 / 8) //MAGENTA
        color_id = al::viz::MAGENTA;
      else if (s >= 3.0 / 16 && s < 3.0 / 8) //RED
        color_id = al::viz::RED;
      else if (s >= -3.0 / 16 && s < 3.0 / 16) //ORANGE
        color_id = al::viz::ORANGE;
      else if (s >= -3.0 / 8 && s < -3.0 / 16) //YELLOW
        color_id = al::viz::YELLOW;
      else if (s >= -5.0 / 8 && s < -3.0 / 8) //GREEN
        color_id = al::viz::GREEN;
      else if (s >= -1 && s < -5.0 / 8) //CYAN
        color_id = al::viz::CYAN;
    }
    else
    {
      color_id = al::viz::GREY;
    }
  }
  return true;
}

//sampling_ration by default 0.5mm/sample
bool
al::math::labelCloudCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data, int label_space,
                                float sampling_ratio)
{

  for (uint i = 0; i < pointcloud_data->points.size (); i++)
    labelPointCurvature (pointcloud_data, i, label_space, sampling_ratio);
  return true;
}
//*/
std::string
al::naming::createFrameName (std::string part, int user_id)
{
  std::stringstream s;
  s << user_id;
  return part + "_" + s.str ();
}

std::string
al::naming::createJointName (std::string part_parent, std::string part_child, int dimension)
{
  std::string s_dimension;
  std::stringstream s;
  s << dimension;
  return part_parent + "_" + part_child + "_" + s.str ();
}

std::string
al::naming::createJointName (std::string part_parent, std::string part_child, std::string dim)
{
  return part_parent + "_" + part_child + "_" + dim;
}

std::string
al::naming::getPartName (std::string frame_name)
{
  size_t pos = frame_name.find_last_of ("_");
  return frame_name.substr (0, pos);
}
