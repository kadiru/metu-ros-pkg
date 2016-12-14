/*
 * request_behavior.cpp
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

#include "al_utils/al_utils.h"
#include "al_msgs/Behavior.h"
#include "al_srvs/GetBehavior.h"
#include "sstream"
#include "iostream"
#include "curses.h"

using namespace std;

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "request_behavior");
  ros::NodeHandle nh_;
  ros::ServiceClient srv_cl_behavior_;
  srv_cl_behavior_ = nh_.serviceClient<al_srvs::GetBehavior> ("/behavior", true);
  srv_cl_behavior_.waitForExistence ();

  al::system::changeInputMode (1);
  while (nh_.ok ())
  {
    char c_id;
    if (al::system::getKey (c_id))
    {
      al_srvs::GetBehaviorRequest req;
      std::string str = std::string (&c_id);
      std::stringstream s (str);
      int i;
      s >> i;
      req.behavior.behavior = i;
      if (req.behavior.behavior == al_msgs::Behavior::REACH)
      {
        std::cout << "Reach behavior is selected" << std::endl;
        std::cout << "Now, enter the object id" << std::endl;
        //        c_id = getch();
        cin >> c_id;
        req.behavior.arg = atoi (&c_id);
        std::cout << "requested behavior: \n" << req.behavior << std::endl;
        al_srvs::GetBehaviorResponse res;
        srv_cl_behavior_.call (req, res);
//        std::cout << "is successful: " << (int)res.successful << std::endl;
      }
      else if (req.behavior.behavior == al_msgs::Behavior::TUCK_ARMS)
      {
        std::cout << "Tuck arms behavior is selected" << std::endl;
        std::cout << "requested behavior: \n" << req.behavior << std::endl;
        al_srvs::GetBehaviorResponse res;
        srv_cl_behavior_.call (req, res);
//        std::cout << "is successful: " << (int)res.successful << std::endl;
      }
    }

    ros::spinOnce ();
  }
  al::system::changeInputMode (0);

  return 0;
}
