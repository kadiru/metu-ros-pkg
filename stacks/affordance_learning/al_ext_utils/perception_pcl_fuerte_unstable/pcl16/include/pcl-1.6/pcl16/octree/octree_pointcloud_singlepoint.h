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
 * $Id: octree_pointcloud_singlepoint.h 6119 2012-07-03 18:50:04Z aichim $
 */

#ifndef OCTREE_SINGLE_POINT_H
#define OCTREE_SINGLE_POINT_H

#include "octree_pointcloud.h"

#include "octree_base.h"
#include "octree2buf_base.h"

#include "octree_nodes.h"

namespace pcl16
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud single point class
     *  \note This pointcloud octree class generate an octrees from a point cloud (zero-copy). Every leaf node contains a single point index from the dataset given by \a setInputCloud.
     *  \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \ingroup octree
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT = OctreeContainerDataT<int>,
        typename BranchT = OctreeContainerEmpty<int>,
        typename OctreeT = OctreeBase<int, LeafT, BranchT> >

    class OctreePointCloudSinglePoint : public OctreePointCloud<PointT, LeafT,
        BranchT, OctreeT>
    {

      public:
        // public typedefs for single/double buffering
        typedef OctreePointCloudSinglePoint<PointT, LeafT, BranchT,
            OctreeBase<int, LeafT, BranchT> > SingleBuffer;
        typedef OctreePointCloudSinglePoint<PointT, LeafT, BranchT,
            Octree2BufBase<int, LeafT, BranchT> > DoubleBuffer;

        /** \brief Constructor.
         *  \param resolution_arg: octree resolution at lowest octree level
         * */
        OctreePointCloudSinglePoint (const double resolution_arg) :
            OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (resolution_arg)
        {
        }

        /** \brief Empty class constructor. */
        virtual ~OctreePointCloudSinglePoint ()
        {
        }

    };

  }
}

#define PCL16_INSTANTIATE_OctreePointCloudSinglePoint(T) template class PCL16_EXPORTS pcl16::octree::OctreePointCloudSinglePoint<T>;

#endif