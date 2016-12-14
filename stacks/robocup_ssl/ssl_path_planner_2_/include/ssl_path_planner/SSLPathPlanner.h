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

#include "nav_msgs/Path.h"
#include "ssl_msgs/PoseControl.h"
#include "ssl_msgs/GlobalState.h"
#include "ssl_msgs/TeamPoseStates.h"
#include "ssl_common/ssl_common.hpp"
#include "ssl_comm_primitives/SSLGlobalStateListener.h"

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
typedef CGAL::Point_2<Kernel> Point_2;

struct Obstacle
{
  int type;
  int team;
  int id;
  std::vector<Circle_2> circles;
  std::vector<Polygon_2> polygons;
};

class SSLPathPlanner : public SSLGlobalStateListener
{
public:
  SSLPathPlanner (ros::NodeHandle& n);
  virtual
  ~SSLPathPlanner ();

  void
  run ();

private:
  ros::NodeHandle nh;

  std::vector<ros::Publisher> pub_robot_path_plans;
  std::vector<nav_msgs::Path> path_plans;
  ros::Subscriber sub_pose_control;

  //used by checkPlanForRobot method
  bool update_pose_controls[MAX_N_ROBOTS_PER_TEAM];
  StateManifoldPtr manifold;
  SimpleSetup* planner_setup;
  PlannerData planner_data_for_robots[MAX_N_ROBOTS_PER_TEAM];
  std::vector<State*> solution_data_for_robots[MAX_N_ROBOTS_PER_TEAM];
  bool pose_ctrl_rcvd;

  uint8_t team_;
  uint8_t tmp_robot_id_queried;
  std::vector<Obstacle> obstacles_;

  void
  init ();



  ros::Publisher pub_team_path_states;

  ros::Subscriber sub_manual_pose_control;

  ssl_msgs::TeamPoseStates team_pose_states;
  ssl_msgs::GlobalState global_st_;
  ssl_msgs::PoseControl pose_control;

  //TODO these can be read an edited from parameter server
  ssl::ROBOT_CONTROL robots_control_type[MAX_N_ROBOTS_PER_TEAM];











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
  globalStateRcvd (const ssl_msgs::GlobalState::ConstPtr &global_msg);

  void
  poseControlRcvd (const ssl_msgs::PoseControl::ConstPtr &pose_msg);

  void
  manualPoseControlRcvd (const ssl_msgs::RobotPoseControl::ConstPtr &pose_msg);

  void
  updateObstacles ();

  bool
  doesIntersectObstacles (const int& id, const Point_2& center);

  bool
  isStateValid (const State* state);

  bool
  doPlanForRobot (const int& id);

  bool
  checkPlanForRobot (const int& id);

  bool
  exePlanForRobot (const int& id);

  void
  showPlanForRobot (const int& id);

  void
  sendPlanForRobot (const int& id);

  void
  initRobot (const int& id);

  void
  runRobot (const int& id);

  void
  show ();
  
  bool
  update();

};

#endif /* SSLPATHPLANNER_H_ */
