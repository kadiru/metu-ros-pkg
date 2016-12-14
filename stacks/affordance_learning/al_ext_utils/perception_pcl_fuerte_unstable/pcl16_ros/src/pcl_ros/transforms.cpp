/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include <sensor_msgs/PointCloud2.h>
#include <pcl16/common/io.h>
#include <pcl16/point_types.h>
#include "pcl16_ros/transforms.h"
#include "pcl16_ros/impl/transforms.hpp"

namespace pcl16_ros
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
transformPointCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, 
                     sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener)
{
  if (in.header.frame_id == target_frame)
  {
    out = in;
    return (true);
  }

  // Get the TF transform
  tf::StampedTransform transform;
  try
  {
    tf_listener.lookupTransform (target_frame, in.header.frame_id, in.header.stamp, transform);
  }
  catch (tf::LookupException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  catch (tf::ExtrapolationException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }

  // Convert the TF transform to Eigen format
  Eigen::Matrix4f eigen_transform;
  transformAsMatrix (transform, eigen_transform);

  transformPointCloud (eigen_transform, in, out);

  out.header.frame_id = target_frame;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
transformPointCloud (const std::string &target_frame, const tf::Transform &net_transform,
                     const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out)
{
  if (in.header.frame_id == target_frame)
  {
    out = in;
    return;
  }

  // Get the transformation
  Eigen::Matrix4f transform;
  transformAsMatrix (net_transform, transform);

  transformPointCloud (transform, in, out);

  out.header.frame_id = target_frame;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
transformPointCloud (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,
                     sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl16::getFieldIndex (in, "x");
  int y_idx = pcl16::getFieldIndex (in, "y");
  int z_idx = pcl16::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }

  // Check if distance is available
  int dist_idx = pcl16::getFieldIndex (in, "distance");

  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i)
  {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;
    
    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
    {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr))  // Invalid point
      {
        pt_out = pt;
      }
      else  // max range point
      {
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    }
    else
    {
      pt_out = transform * pt;
    }

    if (max_range_point)
    {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));
  
    
    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl16::getFieldIndex (in, "vp_x");
  if (vp_idx != -1)
  {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i)
    {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
  double mv[12];
  bt.getBasis ().getOpenGLSubMatrix (mv);

  tf::Vector3 origin = bt.getOrigin ();

  out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];
                                                                     
  out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  out_mat (0, 3) = origin.x ();
  out_mat (1, 3) = origin.y ();
  out_mat (2, 3) = origin.z ();
}

} // namespace pcl16_ros

//////////////////////////////////////////////////////////////////////////////////////////////
template void pcl16_ros::transformPointCloudWithNormals<pcl16::PointNormal> (const pcl16::PointCloud <pcl16::PointNormal> &, pcl16::PointCloud <pcl16::PointNormal> &, const tf::Transform &);
template void pcl16_ros::transformPointCloudWithNormals<pcl16::PointXYZRGBNormal> (const pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, const tf::Transform &);
template void pcl16_ros::transformPointCloudWithNormals<pcl16::PointXYZINormal> (const pcl16::PointCloud <pcl16::PointXYZINormal> &, pcl16::PointCloud <pcl16::PointXYZINormal> &, const tf::Transform &);

//////////////////////////////////////////////////////////////////////////////////////////////
template bool pcl16_ros::transformPointCloudWithNormals<pcl16::PointNormal> (const std::string &, const pcl16::PointCloud<pcl16::PointNormal> &, pcl16::PointCloud<pcl16::PointNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloudWithNormals<pcl16::PointXYZRGBNormal> (const std::string &, const pcl16::PointCloud<pcl16::PointXYZRGBNormal> &, pcl16::PointCloud<pcl16::PointXYZRGBNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloudWithNormals<pcl16::PointXYZINormal> (const std::string &, const pcl16::PointCloud<pcl16::PointXYZINormal> &, pcl16::PointCloud<pcl16::PointXYZINormal> &, const tf::TransformListener &);

//////////////////////////////////////////////////////////////////////////////////////////////
template bool pcl16_ros::transformPointCloudWithNormals<pcl16::PointNormal> (const std::string &, const ros::Time &, const pcl16::PointCloud<pcl16::PointNormal> &, const std::string &, pcl16::PointCloud <pcl16::PointNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloudWithNormals<pcl16::PointXYZRGBNormal> (const std::string &, const ros::Time &, const pcl16::PointCloud<pcl16::PointXYZRGBNormal> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloudWithNormals<pcl16::PointXYZINormal> (const std::string &, const ros::Time &, const pcl16::PointCloud<pcl16::PointXYZINormal> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZINormal> &, const tf::TransformListener &);

//////////////////////////////////////////////////////////////////////////////////////////////
template void pcl16_ros::transformPointCloud<pcl16::PointXYZ> (const pcl16::PointCloud <pcl16::PointXYZ> &, pcl16::PointCloud <pcl16::PointXYZ> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointXYZI> (const pcl16::PointCloud <pcl16::PointXYZI> &, pcl16::PointCloud <pcl16::PointXYZI> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointXYZRGBA> (const pcl16::PointCloud <pcl16::PointXYZRGBA> &, pcl16::PointCloud <pcl16::PointXYZRGBA> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointXYZRGB> (const pcl16::PointCloud <pcl16::PointXYZRGB> &, pcl16::PointCloud <pcl16::PointXYZRGB> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::InterestPoint> (const pcl16::PointCloud <pcl16::InterestPoint> &, pcl16::PointCloud <pcl16::InterestPoint> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointNormal> (const pcl16::PointCloud <pcl16::PointNormal> &, pcl16::PointCloud <pcl16::PointNormal> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointXYZRGBNormal> (const pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointXYZINormal> (const pcl16::PointCloud <pcl16::PointXYZINormal> &, pcl16::PointCloud <pcl16::PointXYZINormal> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointWithRange> (const pcl16::PointCloud <pcl16::PointWithRange> &, pcl16::PointCloud <pcl16::PointWithRange> &, const tf::Transform &);
template void pcl16_ros::transformPointCloud<pcl16::PointWithViewpoint> (const pcl16::PointCloud <pcl16::PointWithViewpoint> &, pcl16::PointCloud <pcl16::PointWithViewpoint> &, const tf::Transform &);

//////////////////////////////////////////////////////////////////////////////////////////////
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZ> (const std::string &, const pcl16::PointCloud <pcl16::PointXYZ> &, pcl16::PointCloud <pcl16::PointXYZ> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZI> (const std::string &, const pcl16::PointCloud <pcl16::PointXYZI> &, pcl16::PointCloud <pcl16::PointXYZI> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZRGBA> (const std::string &, const pcl16::PointCloud <pcl16::PointXYZRGBA> &, pcl16::PointCloud <pcl16::PointXYZRGBA> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZRGB> (const std::string &, const pcl16::PointCloud <pcl16::PointXYZRGB> &, pcl16::PointCloud <pcl16::PointXYZRGB> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::InterestPoint> (const std::string &, const pcl16::PointCloud <pcl16::InterestPoint> &, pcl16::PointCloud <pcl16::InterestPoint> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointNormal> (const std::string &, const pcl16::PointCloud <pcl16::PointNormal> &, pcl16::PointCloud <pcl16::PointNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZRGBNormal> (const std::string &, const pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZINormal> (const std::string &, const pcl16::PointCloud <pcl16::PointXYZINormal> &, pcl16::PointCloud <pcl16::PointXYZINormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointWithRange> (const std::string &, const pcl16::PointCloud <pcl16::PointWithRange> &, pcl16::PointCloud <pcl16::PointWithRange> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointWithViewpoint> (const std::string &, const pcl16::PointCloud <pcl16::PointWithViewpoint> &, pcl16::PointCloud <pcl16::PointWithViewpoint> &, const tf::TransformListener &);

//////////////////////////////////////////////////////////////////////////////////////////////
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZ> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointXYZ> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZ> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZI> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointXYZI> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZI> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZRGBA> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointXYZRGBA> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZRGBA> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZRGB> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointXYZRGB> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZRGB> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::InterestPoint> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::InterestPoint> &, const std::string &, pcl16::PointCloud <pcl16::InterestPoint> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointNormal> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointNormal> &, const std::string &, pcl16::PointCloud <pcl16::PointNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZRGBNormal> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZRGBNormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointXYZINormal> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointXYZINormal> &, const std::string &, pcl16::PointCloud <pcl16::PointXYZINormal> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointWithRange> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointWithRange> &, const std::string &, pcl16::PointCloud <pcl16::PointWithRange> &, const tf::TransformListener &);
template bool pcl16_ros::transformPointCloud<pcl16::PointWithViewpoint> (const std::string &, const ros::Time &, const pcl16::PointCloud <pcl16::PointWithViewpoint> &, const std::string &, pcl16::PointCloud <pcl16::PointWithViewpoint> &, const tf::TransformListener &);

