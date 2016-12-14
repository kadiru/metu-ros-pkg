/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
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
 * $Id: point_types.h 6126 2012-07-03 20:19:58Z aichim $
 *
 */
#ifndef PCL16_DATA_TYPES_H_
#define PCL16_DATA_TYPES_H_

#include <pcl16/pcl_macros.h>
#include <pcl16/common/eigen.h>
#include <bitset>
#include <vector>
#include <pcl16/ros/register_point_struct.h>

/**
  * \file pcl/point_types.h
  * Defines all the PCL implemented PointT point type structures
  * \ingroup common
  */

// We're doing a lot of black magic with Boost here, so disable warnings in Maintainer mode, as we will never
// be able to fix them anyway
#pragma warning(disable: 4201)
//#pragma warning(push, 1)
#ifdef BUILD_Maintainer
#  if defined __GNUC__
#    include <features.h>
#    if __GNUC_PREREQ(4, 3)
#      pragma GCC diagnostic ignored "-Weffc++"
#      pragma GCC diagnostic ignored "-pedantic"
#    else
#      pragma GCC system_header
#    endif
//#  elif defined _MSC_VER
#  endif
#endif

/** @{*/
namespace pcl16
{
  /** \brief Members: float x, y, z
    * \ingroup common
    */
  struct PointXYZ;

  /** \brief Members: rgba
    * \ingroup common
    */
  struct RGB;

  /** \brief Members: float x, y, z, intensity
    * \ingroup common
    */
  struct PointXYZI;

  /** \brief Members: float x, y, z, uin32_t label
    * \ingroup common
    */
  struct PointXYZL;

  /** \brief Members: uint32_t label
    * \ingroup common
    */
  struct Label;

  /** \brief Members: float x, y, z; uint32_t rgba
    * \ingroup common
    */
  struct PointXYZRGBA;

  /** \brief Members: float x, y, z, rgb
    * \ingroup common
    */
  struct PointXYZRGB;

  /** \brief Members: float x, y, z, rgb, uint32_t label
    * \ingroup common
    */
  struct PointXYZRGBL;

  /** \brief Members: float x, y, z, h, s, v
    * \ingroup common
    */
  struct PointXYZHSV;

  /** \brief Members: float x, y
    * \ingroup common
    */
  struct PointXY;

  /** \brief Members: float x, y, z, strength
    * \ingroup common
    */
  struct InterestPoint;

  /** \brief Members: float normal[3], curvature
    * \ingroup common
    */
  struct Normal;

  /** \brief Members: float normal[3]
    * \ingroup common
    */
  struct Axis;

  /** \brief Members: float x, y, z; float normal[3], curvature
    * \ingroup common
    */
  struct PointNormal;

  /** \brief Members: float x, y, z, rgb, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZRGBNormal;

  /** \brief Members: float x, y, z, intensity, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZINormal;

  /** \brief Members: float x, y, z (union with float point[4]), range
    * \ingroup common
    */
  struct PointWithRange;

  /** \brief Members: float x, y, z, vp_x, vp_y, vp_z
    * \ingroup common
    */
  struct PointWithViewpoint;

  /** \brief Members: float j1, j2, j3
    * \ingroup common
    */
  struct MomentInvariants;

  /** \brief Members: float r_min, r_max
    * \ingroup common
    */
  struct PrincipalRadiiRSD;

  /** \brief Members: uint8_t boundary_point
    * \ingroup common
    */
  struct Boundary;

  /** \brief Members: float principal_curvature[3], pc1, pc2
    * \ingroup common
    */
  struct PrincipalCurvatures;

  /** \brief Members: std::vector<float> descriptor, rf[9]
    * \ingroup common
    * \deprecated USE SHOT352 FOR SHAPE AND SHOT1344 FOR SHAPE+COLOR INSTEAD
    */
  struct
  PCL16_DEPRECATED_CLASS (SHOT, "USE SHOT352 FOR SHAPE AND SHOT1344 FOR SHAPE+COLOR INSTEAD");

  /** \brief Members: float descriptor[352], rf[9]
    * \ingroup common
    */
  struct SHOT352;

  /** \brief Members: float descriptor[1344], rf[9]
    * \ingroup common
    */
  struct SHOT1344;

  /** \brief Members: Axis x_axis, y_axis, z_axis
    * \ingroup common
    */
  struct ReferenceFrame;

  /** \brief Members: std::vector<float> descriptor, rf[9]
    * \ingroup common
    */
  struct ShapeContext;

  /** \brief Members: float pfh[125]
    * \ingroup common
    */
  struct PFHSignature125;

  /** \brief Members: float pfhrgb[250]
    * \ingroup common
    */
  struct PFHRGBSignature250;

  /** \brief Members: float f1, f2, f3, f4, alpha_m
    * \ingroup common
    */
  struct PPFSignature;

  /** \brief Members: float f1, f2, f3, f4, r_ratio, g_ratio, b_ratio, alpha_m
    * \ingroup common
    */
  struct PPFRGBSignature;

  /** \brief Members: float values[12]
    * \ingroup common
    */
  struct NormalBasedSignature12;

  /** \brief Members: float fpfh[33]
    * \ingroup common
    */
  struct FPFHSignature33;

  /** \brief Members: float vfh[308]
    * \ingroup common
    */
  struct VFHSignature308;
  /** \brief Members: float esf[640]
    * \ingroup common
    */
  struct ESFSignature640;
  /** \brief Members: float x, y, z, roll, pitch, yaw; float descriptor[36]
    * \ingroup common
    */

  struct Narf36;

  /** \brief Data type to store extended information about a transition from foreground to backgroundSpecification of the fields for BorderDescription::traits.
    * \ingroup common
    */
  typedef std::bitset<32> BorderTraits;

  /** \brief Specification of the fields for BorderDescription::traits.
    * \ingroup common
    */
  enum BorderTrait
  {
    BORDER_TRAIT__OBSTACLE_BORDER, BORDER_TRAIT__SHADOW_BORDER, BORDER_TRAIT__VEIL_POINT,
    BORDER_TRAIT__SHADOW_BORDER_TOP, BORDER_TRAIT__SHADOW_BORDER_RIGHT, BORDER_TRAIT__SHADOW_BORDER_BOTTOM,
    BORDER_TRAIT__SHADOW_BORDER_LEFT, BORDER_TRAIT__OBSTACLE_BORDER_TOP, BORDER_TRAIT__OBSTACLE_BORDER_RIGHT,
    BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM, BORDER_TRAIT__OBSTACLE_BORDER_LEFT, BORDER_TRAIT__VEIL_POINT_TOP,
    BORDER_TRAIT__VEIL_POINT_RIGHT, BORDER_TRAIT__VEIL_POINT_BOTTOM, BORDER_TRAIT__VEIL_POINT_LEFT
  };

  /** \brief Members: int x, y; BorderTraits traits
    * \ingroup common
    */
  struct BorderDescription;

  /** \brief Members: float gradient[3]
    * \ingroup common
    */
  struct IntensityGradient;

  /** \brief Members: float histogram[N]
    * \ingroup common
    */
  template<int N>
  struct Histogram;

  /** \brief Members: float x, y, z, scale
    * \ingroup common
    */
  struct PointWithScale;

  /** \brief Members: float x, y, z, normal[3], rgba, radius, confidence, curvature
    * \ingroup common
    */
  struct PointSurfel;
}

/** @} */

#include <pcl16/impl/point_types.hpp>  // Include struct definitions

// ==============================
// =====PCL16_POINT_CLOUD_REGISTER=====
// ==============================

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::RGB,
    (uint32_t, rgba, rgba)
)
PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointXYZ,
    (float, x, x)
    (float, y, y)
    (float, z, z)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointXYZ, pcl16::_PointXYZ)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointXYZRGBA,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, rgba, rgba)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointXYZRGBA, pcl16::_PointXYZRGBA)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointXYZRGB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointXYZRGB, pcl16::_PointXYZRGB)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointXYZRGBL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, rgba, rgba)
    (uint32_t, label, label)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointXYZRGBL, pcl16::_PointXYZRGBL)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointXYZHSV,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, h, h)
    (float, s, s)
    (float, v, v)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointXYZHSV, pcl16::_PointXYZHSV)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PointXY,
    (float, x, x)
    (float, y, y)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::InterestPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, strength, strength)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointXYZI,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointXYZI, pcl16::_PointXYZI)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PointXYZL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, label, label)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::Label,
    (uint32_t, label, label)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_Normal,
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::Normal, pcl16::_Normal)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_Axis,
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::Axis, pcl16::_Axis)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PointNormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
)
PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointXYZRGBNormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointXYZRGBNormal, pcl16::_PointXYZRGBNormal)
PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PointXYZINormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
)
PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PointWithRange,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, range, range)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_PointWithViewpoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, vp_x, vp_x)
    (float, vp_y, vp_y)
    (float, vp_z, vp_z)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::PointWithViewpoint, pcl16::_PointWithViewpoint)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::MomentInvariants,
    (float, j1, j1)
    (float, j2, j2)
    (float, j3, j3)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PrincipalRadiiRSD,
    (float, r_min, r_min)
    (float, r_max, r_max)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::Boundary,
    (uint8_t, boundary_point, boundary_point)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PrincipalCurvatures,
    (float, principal_curvature_x, principal_curvature_x)
    (float, principal_curvature_y, principal_curvature_y)
    (float, principal_curvature_z, principal_curvature_z)
    (float, pc1, pc1)
    (float, pc2, pc2)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PFHSignature125,
    (float[125], histogram, pfh)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PFHRGBSignature250,
    (float[250], histogram, pfhrgb)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PPFSignature,
    (float, f1, f1)
    (float, f2, f2)
    (float, f3, f3)
    (float, f4, f4)
    (float, alpha_m, alpha_m)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PPFRGBSignature,
    (float, f1, f1)
    (float, f2, f2)
    (float, f3, f3)
    (float, f4, f4)
    (float, r_ratio, r_ratio)
    (float, g_ratio, g_ratio)
    (float, b_ratio, b_ratio)
    (float, alpha_m, alpha_m)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::NormalBasedSignature12,
    (float[12], values, values)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::SHOT352,
    (float[352], descriptor, shot)
    (float[9], rf, rf)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::SHOT1344,
    (float[1344], descriptor, shot)
    (float[9], rf, rf)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::FPFHSignature33,
    (float[33], histogram, fpfh)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::VFHSignature308,
    (float[308], histogram, vfh)
)
PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::ESFSignature640,
    (float[640], histogram, esf)
)
PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::Narf36,
    (float[36], descriptor, descriptor)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::GFPFHSignature16,
    (float[16], histogram, gfpfh)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::IntensityGradient,
    (float, gradient_x, gradient_x)
    (float, gradient_y, gradient_y)
    (float, gradient_z, gradient_z)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::PointWithScale,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, scale, scale)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT(pcl16::PointSurfel,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (uint32_t, rgba, rgba)
    (float, radius, radius)
    (float, confidence, confidence)
    (float, curvature, curvature)
)

PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl16::_ReferenceFrame,
    (float[3], x_axis, x_axis)
    (float[3], y_axis, y_axis)
    (float[3], z_axis, z_axis)
    //(float, confidence, confidence)
)
PCL16_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl16::ReferenceFrame, pcl16::_ReferenceFrame)

//PCL16_POINT_CLOUD_REGISTER_POINT_STRUCT(pcl16::BorderDescription,
//                                  (int, x, x)
//                                  (int, y, y)
//                                  (uint32_t, traits, traits)
//)

namespace pcl16 {
  // Allow float 'rgb' data to match to the newer uint32 'rgba' tag. This is so
  // you can load old 'rgb' PCD files into e.g. a PointCloud<PointXYZRGBA>.
  template<typename PointT>
  struct FieldMatches<PointT, fields::rgba>
  {
    bool operator() (const sensor_msgs::PointField& field)
    {
      if (field.name == "rgb")
      {
        return (field.datatype == sensor_msgs::PointField::FLOAT32 &&
                field.count == 1);
      }
      else
      {
        return (field.name == traits::name<PointT, fields::rgba>::value &&
                field.datatype == traits::datatype<PointT, fields::rgba>::value &&
                field.count == traits::datatype<PointT, fields::rgba>::size);
      }
    }
  };
} // namespace pcl16

#pragma warning(default: 4201)
//#pragma warning(pop)
#ifdef BUILD_Maintainer
#  if defined __GNUC__
#    if __GNUC_PREREQ(4, 3)
#      pragma GCC diagnostic warning "-Weffc++"
#      pragma GCC diagnostic warning "-pedantic"
#    endif
//#  elif defined _MSC_VER
#  endif
#endif

#endif  //#ifndef PCL16_DATA_TYPES_H_
