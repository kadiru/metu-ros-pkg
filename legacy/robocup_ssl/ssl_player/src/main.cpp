/*
 * main.cpp
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
 * IEEE METU Robotics and Automation Society (IEEE METU RAS)
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
 *     * Neither the name of IEEE METU RAS nor the names of its
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

#include "ssl_player/SSLRealPlayer.h"
#include "ssl_player/SSLWSimPlayer.h"

int
main (int argc, char** argv)
{

  //TODO: how webots is going to run nodes?
  //creating a script file that runs a roslaunch command,
  //which is gonna be inside the controllers folder?
  //can this be created automatically from the Makefile?
  //Actually we don't need to do that, just roslaunch scripts
  //inside the controller folder, with the very same name of the
  //module, and give the corresponding arguments through webots

  ros::init (argc, argv, "hedehodo");
  ros::NodeHandle* nh = new ros::NodeHandle("~");
  ssl::SSLPlayer* player;

  bool is_sim = true;
  if(nh->getParam("is_sim", is_sim))
  {
    if(is_sim)
       player = new ssl::SSLWSimPlayer(nh);
    else
      player = new ssl::SSLRealPlayer(nh);

    player->run();
  }
  else
  {
    ROS_ERROR("problem with is_sim parameter, exiting!");
    exit(-1);
  }

  return 0;
}
