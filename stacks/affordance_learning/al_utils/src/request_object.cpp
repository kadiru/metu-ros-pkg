/*
 * request_object.cpp
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

#include "al_srvs/GetCollisionObject.h"
#include "al_utils/al_utils.h"

#include "iostream"
#include "string"
#include "ros/ros.h"

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "request_object");
  ros::NodeHandle nh_;
  ros::ServiceClient srv_cl_get_col_obj_;
  //TODO: proper service client initialization ? auto-reconnect also might be necessary
  srv_cl_get_col_obj_ = nh_.serviceClient<al_srvs::GetCollisionObject> ("/get_collision_object", true);
  srv_cl_get_col_obj_.waitForExistence ();
  srv_cl_get_col_obj_ = nh_.serviceClient<al_srvs::GetCollisionObject> ("/get_collision_object", true);

  std::cout << "Enter an object id to request!" << std::endl;
  al::system::changeInputMode (1);
  while (nh_.ok ())
  {
    char c_id;
    if (al::system::getKey (c_id))
    {
      al_srvs::GetCollisionObjectRequest req;
      req.id = std::string (&c_id);
      std::cout << req.id << std::endl;
      std::cout << "is latest scene required?" << std::endl;
      std::cin >> c_id;
      if (!atoi (&c_id))
        req.latest_scene_required = false;
      else
        req.latest_scene_required = true;
      al_srvs::GetCollisionObjectResponse res;
      srv_cl_get_col_obj_.call (req, res);
      std::cout << res.collision_object << std::endl;
      std::cout << "Enter an object id to request!" << std::endl;
    }

    ros::spinOnce ();
  }
  al::system::changeInputMode (0);

  return 0;
}
