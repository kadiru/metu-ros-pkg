/*
 * SSLPathPlanner.h
 *
 *  Created on: Dec 21, 2011
 *      Author: kadir
 */

#ifndef SSLPATHPLANNER_H_
#define SSLPATHPLANNER_H_

#include "ros/ros.h"
#include "arm_navigation_msgs/CollisionObject.h"

#include "vector"

#include "ssl_msgs/GlobalState.h"
#include "ssl_utils/ssl_utils.h"

namespace ssl
{

  class SSLPathPlanner
  {
  public:
    SSLPathPlanner (ros::NodeHandle* nh);
    virtual
    ~SSLPathPlanner ();

  protected:
    ros::NodeHandle* nh_;
    std::vector<arm_navigation_msgs::CollisionObject> collision_objects_;
    ros::ServiceServer srv_;
    ros::Subscriber sub_;

    void
    globalStateCallback (ssl_msgs::GlobalStateConstPtr global_state);
  };

}

#endif /* SSLPATHPLANNER_H_ */
