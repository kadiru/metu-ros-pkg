/*
 * SSLReactivePPlanner.cpp
 *
 *  Created on: Dec 21, 2011
 *      Author: kadir
 */

#include "ssl_path_planner/SSLReactivePPlanner.h"

namespace ssl
{

  SSLReactivePPlanner::SSLReactivePPlanner (ros::NodeHandle* nh):SSLPathPlanner(nh)
  {
    // TODO Auto-generated constructor stub

  }

  SSLReactivePPlanner::~SSLReactivePPlanner ()
  {
    // TODO Auto-generated destructor stub
  }
  bool
  SSLReactivePPlanner::getRobotPPlan (ssl_srvs::GetReactivePathPlan::Request req, ssl_srvs::GetReactivePathPlan::Response res)
  {

  }
}
