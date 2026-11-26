/*
 * SSLPathPlanner.cpp
 *
 *  Created on: Dec 21, 2011
 *      Author: kadir
 */

#include "ssl_path_planner/SSLPathPlanner.h"

namespace ssl
{

  SSLPathPlanner::SSLPathPlanner (ros::NodeHandle* nh) :
    nh_ (nh)
  {
    sub_ = nh_->subscribe (ssl::naming::topic::EST_GLOBAL_STATE.c_str(), 1, SSLPathPlanner::globalStateCallback, this);
  }

  SSLPathPlanner::~SSLPathPlanner ()
  {
    // TODO Auto-generated destructor stub
  }
  void
  SSLPathPlanner::globalStateCallback (ssl_msgs::GlobalStateConstPtr global_state)
  {

  }
}
