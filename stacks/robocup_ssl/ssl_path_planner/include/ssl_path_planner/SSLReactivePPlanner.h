/*
 * SSLReactivePPlanner.h
 *
 *  Created on: Dec 21, 2011
 *      Author: kadir
 */

#ifndef SSLREACTIVEPPLANNER_H_
#define SSLREACTIVEPPLANNER_H_

#include "SSLPathPlanner.h"

#include "ssl_srvs/GetReactivePathPlan.h"

namespace ssl
{
  class SSLReactivePPlanner : public ssl::SSLPathPlanner
  {
  public:
    SSLReactivePPlanner (ros::NodeHandle* nh);
    virtual
    ~SSLReactivePPlanner ();
  protected:
    bool
    getRobotPPlan (ssl_srvs::GetReactivePathPlan::Request req, ssl_srvs::GetReactivePathPlan::Response res);
  };

}

#endif /* SSLREACTIVEPPLANNER_H_ */
