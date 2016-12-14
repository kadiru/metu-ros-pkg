/*  Copyright (C) 2011, Kadir Firat Uyanik 
 *	Middle East Technical University, Kovan Research Lab
 *	kadir@ceng.metu.edu.tr
 *
 *	http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  SSLGamePlanner.h is part of ssl_game_planner.
 *
 *  ssl_game_planner is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ssl_game_planner is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ssl_game_planner. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * By default, SSLGamePlanner waits for
 *      global_state to receive so that it can publish pose_control messages.
 *
 *      pose_control to be received by a path planner so that it can evaluate
 *      current pose and send new pose_control messages.
 *
 * Once initialization is complete, it sends pose_control messages which include
 * the target robot poses that have been updated.
 */
#ifndef SSLGAMEPLANNER_H_
#define SSLGAMEPLANNER_H_

#include "ros/ros.h"
#include "ssl_msgs/GlobalState.h"
#include "ssl_msgs/PoseControl.h"
#include "ssl_msgs/RefboxControl.h"
#include "ssl_common/ssl_common.hpp"
#include "ssl_comm_primitives/SSLGlobalStateListener.h"

#include "ssl_game_planner/game_state.h"

#include "geometry_msgs/Vector3.h"

#define OUR_FIELD 0//left half
#define GOALIE_ROBOT 0
#define KICK_OFF_ROBOT 1


const double BALL_KICKED_THRESH = 0.05;
const double POSE_REACHED = 0.1;
//TODO BALL_RADIUS is problematic in ssl_common, fix it
const double BALL_PRE_POSE_OFFSET = (ssl::config::ROBOT_RADIUS + 0.043) * 2.0;
const double BALL_FETCH_OFFSET = ssl::config::ROBOT_RADIUS + 0.043 - 0.03;

enum PlanIndex
{
  STOP = 0, KICK_OFF, DIRECT_KICK, INDIRECT_KICK, PENALTY_KICK, BACK_AND_FORTH=99

};

enum BaFState
{
  BACK = 0,
  FORTH = 1
};

class SSLGamePlanner : public SSLGlobalStateListener
{
public:
  SSLGamePlanner (ros::NodeHandle& n);

  virtual
  ~SSLGamePlanner ();

  void
  run ();

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_refbox_ctrl_;
  GameState ssl_game_state;

  ssl_msgs::GlobalRobotState team_global_state[ssl::config::TEAM_CAPACITY];
  ssl_msgs::GlobalRobotState opponent_global_state[ssl::config::TEAM_CAPACITY];
  ssl_msgs::BallState ball_state;
  ssl_msgs::GlobalState global_st;
  ssl_msgs::RefboxControl refbox_ctrl_;
  ssl_msgs::RefboxControl prev_refbox_ctrl_;
  uint8_t team_;
  uint8_t field;

  geometry_msgs::Vector3 prev_ball_pos;
  bool prev_ball_pose_set;

  uint8_t prev_plan_index_;
  uint8_t curr_plan_index_;

  std::vector<int> curr_back_and_forth_state;

  bool update_pose_control;
  bool update_refbox_control;
  int32_t cnt;

  ros::Publisher pub_pose_control;
  ssl_msgs::PoseControl pose_control;
  std::vector<geometry_msgs::Pose2D> target_poses;
  std::vector<bool> poses_updated;
  std::vector<bool> flag_ball_pre_pos_reached;
  std::vector<bool> flag_ball_fetch_pos_reached;

  bool pose_ctrl_sent;

  void
  directKick();

  void
  indirectKick();

  double
  orientToOpponentGoal(uint8_t id);

  void
  initBackAndForth ();

  void
  runBackAndForth ();

  bool
  isPlanChanged();

  bool
  isPoseControlUpdated();

  bool
  isBallKicked ();

  void
  init ();

  void
  runBack();

  void
  runForth();

  bool
  update ();

  // evaluates current situation and changes the index of play
  // as it is specified in the play-book, or updates the necessary
  // parameters for the curr_plan_index

  void
  evaluate ();

  void
  attack ();

  double
  getSquaredDistance (geometry_msgs::Point32 p1, geometry_msgs::Point32 p2);

  geometry_msgs::Point32
  getCurrentPosition (uint8_t team, uint8_t id);

  geometry_msgs::Point32
  getCurrentPosition (uint8_t id);

  double
  getAngleBetween (const geometry_msgs::Point32& src_point, const geometry_msgs::Point32& dest_point);

  void
  goToPoint (uint8_t id, double orientation, geometry_msgs::Point32 point);

  void
  goToBall (uint8_t id, double orientation);

  void
  goToBall (uint8_t id, uint8_t face_to_team_mate_id);

  //evaluates and returns true if plan is finished, returns false otherwise
  bool
  evaluatePlan (uint8_t plan_index);

  void
  runPlan (uint8_t plan_index);

  void
  stopAll ();

  void
  kickOff();

  bool
  send ();

  bool
  isPathPlannerConnected ();

  void
  refboxControlRcvd (ssl_msgs::RefboxControl::ConstPtr refbox_ctrl_msg);

  //  bool
  //  isRefbox
};

#endif /* SSLGAMEPLANNER_H_ */
