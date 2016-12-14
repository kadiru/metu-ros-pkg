/*  Copyright (C) 2011, Kadir Firat Uyanik 
 *	Middle East Technical University, Kovan Research Lab
 *	kadir@ceng.metu.edu.tr
 *
 *	http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  SSLPathPlanner.h is part of ssl_path_planner.
 *
 *  ssl_path_planner is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ssl_path_planner is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ssl_path_planner. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SSLPATHPLANNER_H_
#define SSLPATHPLANNER_H_

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Path.h"

#include "visualization_msgs/Marker.h"

#include "ssl_msgs/PoseControl.h"
#include "ssl_msgs/GlobalState.h"
#include "ssl_common/ssl_common.hpp"
#include "ssl_msgs/AllOdomStates.h"

#include "ssl_msgs/RobotPathPlan.h"

#include <ompl/base/SpaceInformation.h>
#include "ompl/base/StateManifold.h"
#include <ompl/base/manifolds/RealVectorStateManifold.h>
#include <ompl/base/ScopedState.h>
#include <ompl/config.h>
#include "ompl/geometric/SimpleSetup.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/pRRT.h>

#include "CGAL/Circle_2.h"
#include "CGAL/Polygon_2.h"
#include <CGAL/Cartesian.h>
#include "CGAL/squared_distance_2.h"

using namespace ompl::base;
using namespace ompl::geometric;

typedef CGAL::Cartesian<double> Kernel;
typedef CGAL::Circle_2<Kernel> Circle_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Segment_2<Kernel> Segment_2;
typedef Kernel::Point_2 Point;
typedef CGAL::Point_2<Kernel> Point_2;
typedef CGAL::Iso_rectangle_2<Kernel> Rectancle_2;

const double POSE_REACHED_DIST = 0.01;
const double CRITICAL_DISTANCE = 0.5;
const double INTER_CRITICAL_DIST = 0.3;
const double VERY_CRITICAL_DIST = 0.05;
const double MAX_LINEAR_VEL = 1.0;
const double MAX_ANGULAR_VEL = 6.0;
const double MAX_LINEAR_ACC = 2.0;
const double INT_LINEAR_VEL = 1.0;
const double CRITICAL_LINEAR_VEL = 0.2;
const double ACC_TUNING = 10.0;
const double EXPONENT = 50;

struct Trajectory
{
  double vx, vy, va;
  bool kicker_on, dribbler_on;

  double eta;

  Trajectory ()
  {
    ;
  }

  Trajectory (double _vx, double _vy, double _va, double _eta = 0.0, bool _kicker_on = false, bool _dribbler_on = false)
  {
    vx = _vx;
    vy = _vy;
    va = _va;
    kicker_on = _kicker_on;
    dribbler_on = _dribbler_on;
    eta = _eta;
  }
};

struct Obstacle
{
  int type;
  int team;
  int id;
  std::vector<Circle_2> circles;
  std::vector<Polygon_2> polygons;
  std::vector<Rectancle_2> rectangles;
};

class SSLPathPlanner
{
public:
  SSLPathPlanner (ros::NodeHandle& n, const int& team);
  virtual
  ~SSLPathPlanner ();

  void
  init ();

  void
  run ();

private:
  ros::NodeHandle nh;

  ros::Publisher vis_pub;
  ros::Publisher pub_team_path_states;

  std::vector<ros::Publisher> pub_vel_cmds;
  std::vector<ros::Publisher> pub_robot_path_plans_;
  std::vector<ros::Publisher> pub_robot_path_plans;

  ros::Subscriber sub_global_state;
  ros::Subscriber sub_pose_control;
  ros::Subscriber sub_manual_pose_control;
  ros::Subscriber sub_all_odoms_;

  std::vector<nav_msgs::Path> path_plans_;
  std::vector<geometry_msgs::Pose2D> next_target_poses;
  std::vector<State*> solution_data_for_robots[ssl::config::TEAM_CAPACITY];
  ssl_msgs::AllOdomStates all_odoms_;

  ssl_msgs::GlobalState global_state;
  ssl_msgs::PoseControl pose_control;

  std::vector<bool> is_plan_done;

  std::vector<ssl_msgs::GlobalRobotState> team_state_;

  //TODO these can be read an edited from parameter server
  ssl::ROBOT_CONTROL robots_control_type[ssl::config::TEAM_CAPACITY];

  //used by checkPlanForRobot method
  bool update_pose_controls[ssl::config::TEAM_CAPACITY];

  StateManifoldPtr manifold;
  SimpleSetup* planner_setup;

  bool global_st_rcvd;
  bool pose_ctrl_rcvd;
  bool odom_rcvd;

  int team_;
  int tmp_robot_id_queried;
  std::vector<Obstacle> obstacles_;

  //this is true or false for each robot, which changes the path being found
  std::vector<bool> flag_ball_obs;

  bool
  isPlanDone(uint8_t id);

  double
  getAngleBetween (const geometry_msgs::Point& src_point, const geometry_msgs::Point& dest_point);

  double
  getSquaredDistance (geometry_msgs::Point p1, geometry_msgs::Point p2);

  double
  getSquaredDistance (Point_2 p1, Point_2 p2);

  void
  markWaypoint (geometry_msgs::Point point, int32_t type, uint32_t robot_index);

  geometry_msgs::Point
  getCurrentPosition (uint8_t id);

  double
  getCurrentAngularVelocity(uint8_t id);

  double
  getAngularDifference (double ref, double rel);

  geometry_msgs::Vector3
  getCurrentVelocity (uint8_t id);

  double
  getVelMagnitude (geometry_msgs::Vector3 vel);

  double
  getVelAngle (geometry_msgs::Vector3 vel);

  void
  rotate (geometry_msgs::Vector3& vec, double angle);

  void
  globalStateRcvd (const ssl_msgs::GlobalState::ConstPtr &global_msg);

  Obstacle
  getGoalAsObstacle (ssl::FIELD_SIDES left_or_right);

  Obstacle
  getGoalAsObstacle2 (ssl::FIELD_SIDES left_or_right);

  //returns the robot as an obstacle considering its current velocity and the acceleration command
  Obstacle
  getRobotAsObstacle (uint8_t team, uint8_t id, const geometry_msgs::Vector3& vel, const geometry_msgs::Vector3& acc);

  //assumes the robot is statical
  Obstacle
  getRobotAsObstacle (uint8_t team, uint8_t id);

  //assumes that the most confident ball is at the zeroth index of the global_state data
  //only adds this ball as obstacle by discarding the other possibilities
  Obstacle
  getBallAsObstacle ();
  //  void
  //  addCircleToRobotObstacle (uint8_t team, uint8_t id);

  geometry_msgs::Vector3
  getRobotPosition (uint8_t team, uint32_t id);

  void
  odomStateRcvd (const ssl_msgs::AllOdomStates::ConstPtr &odoms_msg);

  double
  getSquaredDistance (const State* state_1, const State* state_2);

  double
  getSquaredDistance (const State* state_1, const ScopedState<RealVectorStateManifold>& state_2);

  int
  getNearestOnwardStateIndex (const std::vector<State*>& states,
                              const ScopedState<RealVectorStateManifold>& queried_State);

  int
  getNearestOnwardStateIndex (const std::vector<State*>& states, const State* queried_state);

  int
  getNearestStateIndex (const std::vector<State*>& states, const ScopedState<RealVectorStateManifold>& queried_State);

  int
  getNearestStateIndex (const std::vector<State*>& states, const State* queried_state);

  void
  getTrimmedPath (PathGeometric& trimmed_path, const std::vector<State*>& states, int trim_index);

  void
  poseControlRcvd (const ssl_msgs::PoseControl::ConstPtr pose_msg);

  void
  updateObstacles ();

  bool
  doesIntersectObstacles (const int& id, const Point_2& center);

  bool
  doesIntersectObstacles (const int& id, const Point_2& start, const Point_2& goal);

  bool
  isStateValid (const State* state);

  bool
  doPlanForRobot (const int& id/*, bool& is_stop*/);

  bool
  checkPlanForRobot (const int& id);

  geometry_msgs::Vector3
  exePlanForRobot (const int& id);

  void
  showPlanForRobot (const int& id);

  void
  sendPlanForRobot (const int& id);

  void
  initRobot (const int& id);

  void
  runRobot (const int& id);
};

#endif /* SSLPATHPLANNER_H_ */
