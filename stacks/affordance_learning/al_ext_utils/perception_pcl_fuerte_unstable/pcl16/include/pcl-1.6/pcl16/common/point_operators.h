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
 * $Id: point_operators.h 5355 2012-03-27 23:52:01Z nizar $
 *
 */

#ifndef PCL16_COMMON_POINT_OPERATORS_H
#define PCL16_COMMON_POINT_OPERATORS_H

#include <pcl16/point_types.h>

namespace pcl16
{
  namespace common
  {
    /** \brief provide a set of operator on points
      * Default behaviour is to consider only XYZ component but several specializations
      * are added.
      */
    ///addition operator for PointT
    template <typename PointT> inline PointT
    operator+ (const PointT& lhs, const PointT& rhs)
    {
      PointT result = lhs;
      result.getVector3fMap () += rhs.getVector3fMap ();
      return (result);
    }
    ///subtraction operator for PointT
    template <typename PointT> inline PointT
    operator- (const PointT& lhs, const PointT& rhs)
    {
      PointT result = lhs;
      result.getVector3fMap () -= rhs.getVector3fMap ();
      return (result);
    }
    ///multiplication operator for PointT and a scalar
    template <typename PointT> inline PointT
    operator* (const float& scalar, const PointT& p)
    {
      PointT result = p;
      result.getVector3fMap () *= scalar;
      return (result);
    }
    ///multiplication operator for PointT and a scalar
    template <typename PointT> inline PointT
    operator* (const PointT& p, const float& scalar)
    {
      PointT result = p;
      result.getVector3fMap () *= scalar;
      return (result);
    }
    ///division operator for PointT and a scalar
    template <typename PointT> inline PointT
    operator/ (const float& scalar, const PointT& p)
    {
      PointT result = p;
      result.getVector3fMap () /= scalar;
      return (result);
    }
    ///division operator for PointT and a scalar
    template <typename PointT> inline PointT
    operator/ (const PointT& p, const float& scalar)
    {
      PointT result = p;
      result.getVector3fMap () /= scalar;
      return (result);
    }
    ///plus assign operator for PointT
    template <typename PointT> inline PointT&
    operator+= (PointT& lhs, const PointT& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      return (lhs);
    }
    ///minus assign operator for PointT
    template <typename PointT> inline PointT&
    operator-= (PointT& lhs, const PointT& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      return (lhs);
    }
    ///multiply assign operator for PointT
    template <typename PointT> inline PointT&
    operator*= (PointT& p, const float& scalar)
    {
      p.getVector3fMap () *= scalar;
      return (PointT ());
    }
    ///divide assign operator for PointT
    template <typename PointT> inline PointT&
    operator/= (PointT& p, const float& scalar)
    {
      p.getVector3fMap () /= scalar;
      return (p);
    }

    ///addition operator for PointXYZI
    template <> inline pcl16::PointXYZI
    operator+ (const pcl16::PointXYZI& lhs, const pcl16::PointXYZI& rhs)
    {
      pcl16::PointXYZI result = lhs;
      result.getVector3fMap () += rhs.getVector3fMap ();
      result.intensity += rhs.intensity;
      return (result);
    }
    ///subtraction operator for PointXYZI
    template <> inline pcl16::PointXYZI
    operator- (const pcl16::PointXYZI& lhs, const pcl16::PointXYZI& rhs)
    {
      pcl16::PointXYZI result = lhs;
      result.getVector3fMap () -= rhs.getVector3fMap ();
      result.intensity -= rhs.intensity;
      return (result);
    }
    ///multiplication operator for PointXYZI and a scalar
    template <> inline pcl16::PointXYZI
    operator* (const float& scalar, const pcl16::PointXYZI& p)
    {
      pcl16::PointXYZI result = p;
      result.getVector3fMap () *= scalar;
      result.intensity *= scalar;
      return (result);
    }
    ///multiplication operator for PointXYZI and a scalar
    template <> inline pcl16::PointXYZI
    operator* (const pcl16::PointXYZI& p, const float& scalar)
    {
      pcl16::PointXYZI result = p;
      result.getVector3fMap () *= scalar;
      result.intensity *= scalar;
      return (result);
    }
    ///plus assign operator for PointXYZI
    template <> inline pcl16::PointXYZI&
    operator+= (pcl16::PointXYZI& lhs, const pcl16::PointXYZI& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      lhs.intensity += rhs.intensity;
      return (lhs);
    }
    ///minus assign operator for PointXYZI
    template <> inline pcl16::PointXYZI&
    operator-= (pcl16::PointXYZI& lhs, const pcl16::PointXYZI& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      lhs.intensity -= rhs.intensity;
      return (lhs);
    }
    ///multiply assign operator for PointXYZI
    template <> inline pcl16::PointXYZI&
    operator*= (pcl16::PointXYZI& lhs, const float& scalar)
    {
      lhs.getVector3fMap () *= scalar;
      lhs.intensity *= scalar;
      return (lhs);
    }
    template <> inline pcl16::PointXYZINormal
    operator+ (const pcl16::PointXYZINormal& lhs, const pcl16::PointXYZINormal& rhs)
    {
      pcl16::PointXYZINormal result = lhs;
      result.getVector3fMap () += rhs.getVector3fMap ();
      result.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      result.intensity += rhs.intensity;
      result.curvature += rhs.curvature;
      return (result);
    }

    template <> inline pcl16::PointXYZINormal
    operator- (const pcl16::PointXYZINormal& lhs, const pcl16::PointXYZINormal& rhs)
    {
      pcl16::PointXYZINormal result = lhs;
      result.getVector3fMap () -= rhs.getVector3fMap ();
      result.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      result.intensity -= rhs.intensity;
      result.curvature -= rhs.curvature;
      return (result);
    }

    template <> inline pcl16::PointXYZINormal&
    operator+= (pcl16::PointXYZINormal& lhs, const pcl16::PointXYZINormal& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      lhs.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      lhs.intensity += rhs.intensity;
      lhs.curvature += rhs.curvature;
      return (lhs);
    }

    template <> inline pcl16::PointXYZINormal&
    operator-= (pcl16::PointXYZINormal& lhs, const pcl16::PointXYZINormal& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      lhs.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      lhs.intensity-= rhs.intensity;
      lhs.curvature-= rhs.curvature;
      return (lhs);
    }

    template <> inline pcl16::PointXYZINormal&
    operator*= (pcl16::PointXYZINormal& lhs, const float& scalar)
    {
      lhs.getVector3fMap () *= scalar;
      lhs.getNormalVector3fMap () *= scalar;
      lhs.intensity *= scalar;
      lhs.curvature *= scalar;
      return (lhs);
    }

    template <> inline pcl16::PointXYZINormal
    operator* (const float& scalar, const pcl16::PointXYZINormal& p)
    {
      pcl16::PointXYZINormal result = p;
      result.getVector3fMap () *= scalar;
      result.getNormalVector3fMap () *= scalar;
      result.intensity *= scalar;
      result.curvature *= scalar;
      return (result);
    }

    template <> inline pcl16::PointXYZINormal
    operator* (const pcl16::PointXYZINormal& p, const float& scalar)
    {
      return (operator* (scalar, p));
    }

    ///addition operator for Normal
    template <> inline pcl16::Normal
    operator+ (const pcl16::Normal& lhs, const pcl16::Normal& rhs)
    {
      pcl16::Normal result = lhs;
      result.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      result.curvature += rhs.curvature;
      return (result);
    }
    ///subtraction operator for Normal
    template <> inline pcl16::Normal
    operator- (const pcl16::Normal& lhs, const pcl16::Normal& rhs)
    {
      pcl16::Normal result = lhs;
      result.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      result.curvature -= rhs.curvature;
      return (result);
    }
    ///multiplication operator for Normal and a scalar
    template <> inline pcl16::Normal
    operator* (const float& scalar, const pcl16::Normal& p)
    {
      pcl16::Normal result = p;
      result.getNormalVector3fMap () *= scalar;
      result.curvature *= scalar;
      return (result);
    }
    ///multiplication operator for Normal and a scalar
    template <> inline pcl16::Normal
    operator* (const pcl16::Normal& p, const float& scalar)
    {
      pcl16::Normal result = p;
      result.getNormalVector3fMap () *= scalar;
      result.curvature *= scalar;
      return (result);
    }
    ///plus assign operator for Normal
    template <> inline pcl16::Normal&
    operator+= (pcl16::Normal& lhs, const pcl16::Normal& rhs)
    {
      lhs.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      lhs.curvature += rhs.curvature;
      return (lhs);
    }
    ///minus assign operator for Normal
    template <> inline pcl16::Normal&
    operator-= (pcl16::Normal& lhs, const pcl16::Normal& rhs)
    {
      lhs.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      lhs.curvature -= rhs.curvature;
      return (lhs);
    }
    ///multiply assign operator for Normal
    template <> inline pcl16::Normal&
    operator*= (pcl16::Normal& lhs, const float& scalar)
    {
      lhs.getNormalVector3fMap () *= scalar;
      lhs.curvature *= scalar;
      return (lhs);
    }

    ///addition operator for PointXYZRGB
    template <> inline pcl16::PointXYZRGB
    operator+ (const pcl16::PointXYZRGB& lhs, const pcl16::PointXYZRGB& rhs)
    {
      pcl16::PointXYZRGB result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () += rhs.getVector3fMap ();
      result.r = static_cast<uint8_t> (lhs.r + rhs.r);
      result.g = static_cast<uint8_t> (lhs.g + rhs.g);
      result.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (result);
    }
    ///subtraction operator for PointXYZRGB
    template <> inline pcl16::PointXYZRGB
    operator- (const pcl16::PointXYZRGB& lhs, const pcl16::PointXYZRGB& rhs)
    {
      pcl16::PointXYZRGB result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () -= rhs.getVector3fMap ();
      result.r = static_cast<uint8_t> (lhs.r - rhs.r);
      result.g = static_cast<uint8_t> (lhs.g - rhs.g);
      result.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (result);
    }

    template <> inline pcl16::PointXYZRGB
    operator* (const float& scalar, const pcl16::PointXYZRGB& p)
    {
      pcl16::PointXYZRGB result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.r = static_cast<uint8_t> (scalar * p.r);
      result.g = static_cast<uint8_t> (scalar * p.g);
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl16::PointXYZRGB
    operator* (const pcl16::PointXYZRGB& p, const float& scalar)
    {
      pcl16::PointXYZRGB result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.r = static_cast<uint8_t> (scalar * p.r);
      result.g = static_cast<uint8_t> (scalar * p.g);
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl16::PointXYZRGB&
    operator+= (pcl16::PointXYZRGB& lhs, const pcl16::PointXYZRGB& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (lhs);
    }

    template <> inline pcl16::PointXYZRGB&
    operator-= (pcl16::PointXYZRGB& lhs, const pcl16::PointXYZRGB& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (lhs);
    }

    template <> inline pcl16::PointXYZRGB&
    operator*= (pcl16::PointXYZRGB& lhs, const float& scalar)
    {
      lhs.getVector3fMap () *= scalar;
      lhs.r = static_cast<uint8_t> (lhs.r * scalar);
      lhs.g = static_cast<uint8_t> (lhs.g * scalar);
      lhs.b = static_cast<uint8_t> (lhs.b * scalar);
      return (lhs);
    }

    ///addition operator for PointXYZRGBAA
    template <> inline pcl16::PointXYZRGBA
    operator+ (const pcl16::PointXYZRGBA& lhs, const pcl16::PointXYZRGBA& rhs)
    {
      pcl16::PointXYZRGBA result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () += rhs.getVector3fMap ();
      result.r = static_cast<uint8_t> (lhs.r + rhs.r);
      result.g = static_cast<uint8_t> (lhs.g + rhs.g);
      result.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (result);
    }
    ///subtraction operator for PointXYZRGBA
    template <> inline pcl16::PointXYZRGBA
    operator- (const pcl16::PointXYZRGBA& lhs, const pcl16::PointXYZRGBA& rhs)
    {
      pcl16::PointXYZRGBA result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () -= rhs.getVector3fMap ();
      result.r = static_cast<uint8_t> (lhs.r - rhs.r);
      result.g = static_cast<uint8_t> (lhs.g - rhs.g);
      result.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (result);
    }

    template <> inline pcl16::PointXYZRGBA
    operator* (const float& scalar, const pcl16::PointXYZRGBA& p)
    {
      pcl16::PointXYZRGBA result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.r = static_cast<uint8_t> (scalar * p.r);
      result.g = static_cast<uint8_t> (scalar * p.g);
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl16::PointXYZRGBA
    operator* (const pcl16::PointXYZRGBA& p, const float& scalar)
    {
      pcl16::PointXYZRGBA result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.r = static_cast<uint8_t> (scalar * p.r);
      result.g = static_cast<uint8_t> (scalar * p.g);
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl16::PointXYZRGBA&
    operator+= (pcl16::PointXYZRGBA& lhs, const pcl16::PointXYZRGBA& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (lhs);
    }

    template <> inline pcl16::PointXYZRGBA&
    operator-= (pcl16::PointXYZRGBA& lhs, const pcl16::PointXYZRGBA& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (lhs);
    }

    template <> inline pcl16::PointXYZRGBA&
    operator*= (pcl16::PointXYZRGBA& lhs, const float& scalar)
    {
      lhs.getVector3fMap () *= scalar;
      lhs.r = static_cast<uint8_t> (lhs.r * scalar);
      lhs.g = static_cast<uint8_t> (lhs.g * scalar);
      lhs.b = static_cast<uint8_t> (lhs.b * scalar);
      return (lhs);
    }

    ///addition operator for RGBA
    template <> inline pcl16::RGB
    operator+ (const pcl16::RGB& lhs, const pcl16::RGB& rhs)
    {
      pcl16::RGB result;
      result.r = static_cast<uint8_t> (lhs.r + rhs.r);
      result.g = static_cast<uint8_t> (lhs.g + rhs.g);
      result.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (result);
    }
    ///subtraction operator for RGB
    template <> inline pcl16::RGB
    operator- (const pcl16::RGB& lhs, const pcl16::RGB& rhs)
    {
      pcl16::RGB result;
      result.r = static_cast<uint8_t> (lhs.r - rhs.r);
      result.g = static_cast<uint8_t> (lhs.g - rhs.g);
      result.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (result);
    }

    template <> inline pcl16::RGB
    operator* (const float& scalar, const pcl16::RGB& p)
    {
      pcl16::RGB result;
      result.r = static_cast<uint8_t> (scalar * p.r);
      result.g = static_cast<uint8_t> (scalar * p.g);
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl16::RGB
    operator* (const pcl16::RGB& p, const float& scalar)
    {
      pcl16::RGB result;
      result.r = static_cast<uint8_t> (scalar * p.r);
      result.g = static_cast<uint8_t> (scalar * p.g);
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl16::RGB&
    operator+= (pcl16::RGB& lhs, const pcl16::RGB& rhs)
    {
      lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (lhs);
    }

    template <> inline pcl16::RGB&
    operator-= (pcl16::RGB& lhs, const pcl16::RGB& rhs)
    {
      lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (lhs);
    }

    template <> inline pcl16::RGB&
    operator*= (pcl16::RGB& lhs, const float& scalar)
    {
      lhs.r = static_cast<uint8_t> (lhs.r * scalar);
      lhs.g = static_cast<uint8_t> (lhs.g * scalar);
      lhs.b = static_cast<uint8_t> (lhs.b * scalar);
      return (lhs);
    }
  }
}

#endif
