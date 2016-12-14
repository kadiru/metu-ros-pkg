/*
 * Perception.cpp
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

#include "al_perception/perception/Perception.h"

namespace al
{
  namespace perception
  {
    Perception::Perception (ros::NodeHandle* nh)
    {
      nh_ = nh;
      bool param_perception3D_active = false;
      if (nh_->getParam ("/Perceptors/Visual3D/config/is_active", param_perception3D_active))
      {
        if (param_perception3D_active)
        {
          Visual3D* visual_3d = new Visual3D (nh_);
          perceptors_.push_back (visual_3d);
        }
      }
      else
      {
        //parameter couldn't be read
        ROS_WARN("no such parameter as </Perceptors/Visual3D/config/is_active>");
      }
    }

    Perception::~Perception ()
    {
      // TODO Auto-generated destructor stub
    }

    void
    Perception::init ()
    {
      spinner_ = new ros::AsyncSpinner (8);
      spinner_->start ();

      for (uint8_t i = 0; i < perceptors_.size (); i++)
        perceptors_[i]->init ();

      bool all_ready = true;
      ROS_INFO("waiting for all perceptor dependent services to be ready to start perception");
      do
      {
        for (uint8_t i = 0; i < perceptors_.size (); i++)
        {
          ROS_DEBUG("perceptor %d is %d", (int)i, perceptors_[i]->isReady ());
          all_ready = all_ready && perceptors_[i]->isReady ();
        }
        ros::spinOnce ();
      } while (nh_->ok () && !all_ready);
      ROS_INFO("all perceptor dependent services are ready, perception started!");
    }

    void
    Perception::run ()
    {
      ros::Rate r (30);
      while (nh_->ok ())
      {
        ROS_DEBUG("#perceptors: %d",(int)perceptors_.size());

        al_msgs::FeatureVector msg_features;
        for (uint8_t i = 0; i < perceptors_.size (); i++)
          perceptors_[i]->percept ();

        ros::spinOnce ();
        r.sleep ();
      }
    }
  }
}
