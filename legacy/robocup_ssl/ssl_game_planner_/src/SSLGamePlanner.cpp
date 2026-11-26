/*  Copyright (C) 2011, Kadir Firat Uyanik 
 *	Middle East Technical University, Kovan Research Lab
 *	kadir@ceng.metu.edu.tr
 *
 *	http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  SSLGamePlanner.cpp is part of ssl_game_planner.
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

#include "../include/ssl_game_planner/SSLGamePlanner.h"

void
SSLGamePlanner::refboxControlRcvd (ssl_msgs::RefboxControl::ConstPtr refbox_ctrl_msg)
{
  refbox_ctrl_ = *refbox_ctrl_msg;
  std::cout << refbox_ctrl_.refbox_control << std::endl;
  update_refbox_control = true;
}

SSLGamePlanner::SSLGamePlanner (ros::NodeHandle& n) :
  SSLGlobalStateListener (n)
{
  nh_ = n;
  std::string team_name;
  if (nh_.hasParam ("team"))
    nh_.getParam ("team", team_name);
  else
    team_name = ssl::naming::entity::BLUE_TEAM;//BLUE for default

  if (!team_name.compare (ssl::naming::entity::BLUE_TEAM))
    team_ = (uint8_t)ssl::BLUE;
  else
    team_ = (uint8_t)ssl::YELLOW;

  target_poses.resize (ssl::config::TEAM_CAPACITY);
  poses_updated.resize (ssl::config::TEAM_CAPACITY, false);
  curr_back_and_forth_state.resize (ssl::config::TEAM_CAPACITY, FORTH);

  std::string pose_control_topic_name;
  if (team_ == 0)
    pose_control_topic_name.assign ("B");
  else
    pose_control_topic_name.assign ("Y");
  pose_control_topic_name.append ("_");
  pose_control_topic_name.append (ssl::naming::POSE_CTRL_TOPIC_NAME);

  pub_pose_control = nh_.advertise<ssl_msgs::PoseControl> (pose_control_topic_name, 10);

  prev_plan_index_ = STOP;
  curr_plan_index_ = STOP;

  pose_ctrl_sent = false;
  update_pose_control = false;
  update_refbox_control = false;
  prev_ball_pose_set = false;
  sub_refbox_ctrl_ = nh_.subscribe ("referee_control", 10, &SSLGamePlanner::refboxControlRcvd, this);
}

SSLGamePlanner::~SSLGamePlanner ()
{
  ros::shutdown ();
}

void
SSLGamePlanner::init ()
{
  //init global state
  global_st = SSLGlobalStateListener::global_state_;

  //  ROS_INFO("waiting for global_state to be received");
  while (nh_.ok () && !update ())
  {
    ros::spinOnce ();
  }
}

bool
SSLGamePlanner::update ()
{
  //if globalstate and refbox control didn't change,
  //don't do anything
  if (!isGlobalStateUptodate () && !update_refbox_control)
  {
    //    ROS_INFO("waiting for global_state to be received");
    return false;
  }

  if (!isPathPlannerConnected ())
  {
    //    ROS_INFO("global_state is received, waiting for a path planner");
    return false;
  }

  if (isGlobalStateUptodate ())
  {
    global_st = getGlobalState ();
    for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
    {
      if (team_ == ssl::BLUE)
      {
        team_global_state[i] = global_st.blue_team[i];
        opponent_global_state[i] = global_st.yellow_team[i];
      }
      else
      {
        team_global_state[i] = global_st.yellow_team[i];
        opponent_global_state[i] = global_st.blue_team[i];
      }

      if (global_st.balls.size () > 0)
        ball_state = global_st.balls[0];
    }
  }

  if (update_refbox_control)
  {
    ssl_game_state.transition (refbox_ctrl_.refbox_control, isBallKicked ());
    update_refbox_control = false;
  }

  return true;
}

bool
SSLGamePlanner::isBallKicked ()
{

  if (ssl_game_state.restart ())
  {
    if (global_state_.balls.size () > 0)
    {
      if (!prev_ball_pose_set)
      {
        prev_ball_pos.x = ball_state.position.x;
        prev_ball_pos.y = ball_state.position.y;
        prev_ball_pose_set = true;
        //        std::cout<<"----------BALL POS SET---------"<<std::endl;
        //        std::cout<<ball_state<<std::endl;
        //        std::cout<<"----------BALL POS SET---------"<<std::endl;
      }
    }
  }
  else
    return true;

  if (prev_ball_pose_set)
  {
    if (global_state_.balls.size () > 0)
    {
      std::cout << ball_state << std::endl;
      double dx = ball_state.position.x - prev_ball_pos.x;
      double dy = ball_state.position.y - prev_ball_pos.y;

      double dist = sqrt (dx * dx + dy * dy);
      if (dist > BALL_KICKED_THRESH)
      {
        std::cout << "********** BALL KICKED **********" << std::endl;
        prev_ball_pose_set = false;
        return true;
      }
      else
        return false;
    }
  }
  else
    return false;
}

void
SSLGamePlanner::initBackAndForth ()
{
  for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
    if (team_global_state[i].state != ssl::OUT_OF_FOV)
    {
      curr_back_and_forth_state[i] = FORTH;
      target_poses[i].x = 1.0;
      target_poses[i].y = i * 0.65 - 1.0;
      target_poses[i].theta = ssl::math::PI;
    }
}

void
SSLGamePlanner::runBackAndForth ()
{
  pose_control.pose.clear ();
  for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
  {
    ssl_msgs::RobotPoseControl robot_pose_control;
    if (team_global_state[i].state != ssl::OUT_OF_FOV)
    {
      if (curr_back_and_forth_state[i] == FORTH)
      {
        geometry_msgs::Point32 p = getCurrentPosition (team_, i);
        double dx = target_poses[i].x - p.x;
        double dy = target_poses[i].y - p.y;
        double dist = sqrt (dx * dx + dy * dy);
        if (dist < 0.02)
        {
          curr_back_and_forth_state[i] = BACK;
          target_poses[i].x = -1.0;
          target_poses[i].y = i * 0.65 - 1.0;
          target_poses[i].theta = ssl::math::PI;
        }
      }
      else
      {
        geometry_msgs::Point32 p = getCurrentPosition (team_, i);
        double dx = target_poses[i].x - p.x;
        double dy = target_poses[i].y - p.y;
        double dist = sqrt (dx * dx + dy * dy);
        if (dist < 0.02)
        {
          curr_back_and_forth_state[i] = FORTH;
          target_poses[i].x = 1.0;
          target_poses[i].y = i * 0.65 - 1.0;
          target_poses[i].theta = 0.0;
        }
      }

      robot_pose_control.id = i;
      robot_pose_control.team = team_;
      robot_pose_control.pose.x = target_poses[i].x;
      robot_pose_control.pose.y = target_poses[i].y;
      robot_pose_control.pose.theta = target_poses[i].theta;
      robot_pose_control.flag_ball_obs = true;
      target_poses[robot_pose_control.id] = robot_pose_control.pose;
      pose_control.pose.push_back (robot_pose_control);
      update_pose_control = true;
    }
  }
}

bool
SSLGamePlanner::isPathPlannerConnected ()
{
  if (pub_pose_control.getNumSubscribers () > 0)
    return true;
  else
    return false;
}

double
SSLGamePlanner::getAngleBetween (const geometry_msgs::Point32& src_point, const geometry_msgs::Point32& dest_point)
{
  return atan2 (dest_point.y - src_point.y, dest_point.x - src_point.x);
}

double
SSLGamePlanner::orientToOpponentGoal(uint8_t id)
{
  geometry_msgs::Point32 goal_pos;
  goal_pos.x = -3.025 + 2* 3.025*OUR_FIELD;
  goal_pos.y = 0;
  goal_pos.z = 0;

  geometry_msgs::Point32 ball_pos;
  ball_pos.x = ball_state.position.x;
  ball_pos.y = ball_state.position.y;
  ball_pos.z = ball_state.position.z;

  return getAngleBetween(ball_pos, goal_pos);
}

void
SSLGamePlanner::kickOff ()
{
  if (ssl_game_state.kickoff ())
  {
    if (ssl_game_state.ourKickoff ())
    {
      goToBall ((uint8_t)KICK_OFF_ROBOT, 0.0 + (int)OUR_FIELD * ssl::math::PI);
    }
    else
    {
      geometry_msgs::Point32 p;
      p.x = -(0.5 + ssl::config::ROBOT_RADIUS) + 2 * (0.5 + ssl::config::ROBOT_RADIUS) * (int)OUR_FIELD;
      p.y = 0;
      p.z = 0;
      goToPoint ((uint8_t)KICK_OFF_ROBOT, 0.0 + (int)OUR_FIELD * ssl::math::PI, p);
    }
  }
  geometry_msgs::Point32 p;
  p.x = -2.975 + 2 * 2.975 * (int)OUR_FIELD;
  p.y = 0;
  p.z = 0;
  goToPoint ((uint8_t)GOALIE_ROBOT, 0 + (int)OUR_FIELD * ssl::math::PI, p);

  p.x = -1.000 + 2 * 1.0 * (int)OUR_FIELD;
  p.y = -1.000;
  p.z = 0;
  goToPoint ((uint8_t)2, 0, p);

  p.y = 0.0;

  goToPoint ((uint8_t)3, 0, p);

  p.y = 1.0;

  goToPoint ((uint8_t)4, 0, p);
}

void
SSLGamePlanner::directKick()
{
//TODO:
}

void
SSLGamePlanner::indirectKick()
{
//TODO:
}

void
SSLGamePlanner::runPlan (uint8_t plan_index)
{
  switch (plan_index)
  {
    case BACK_AND_FORTH:
      runBackAndForth ();
      break;
    case STOP:
      stopAll ();
      break;
    case KICK_OFF:
      kickOff ();
      break;
    case DIRECT_KICK:
      break;
    case INDIRECT_KICK:
      break;
    default:
      break;
  }
}

void
SSLGamePlanner::stopAll ()
{
  for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
  {
    if (team_global_state[i].state != ssl::OUT_OF_FOV && !poses_updated[i])
    {
      poses_updated[i] = true;
      target_poses[i].x = team_global_state[i].pose.x;
      target_poses[i].y = team_global_state[i].pose.y;
      target_poses[i].theta = team_global_state[i].pose.theta;

      ssl_msgs::RobotPoseControl robot_pose_control;
      robot_pose_control.id = i;
      robot_pose_control.team = team_;
      robot_pose_control.pose.x = target_poses[i].x;
      robot_pose_control.pose.y = target_poses[i].y;
      robot_pose_control.pose.theta = target_poses[i].theta;
      pose_control.pose.push_back (robot_pose_control);
    }
  }
}

void
SSLGamePlanner::evaluate ()
{
  pose_control.pose.clear ();

  if (!ssl_game_state.canMove ())
    curr_plan_index_ = STOP;
  else
  {
    if (ssl_game_state.kickoff ())
      curr_plan_index_ = KICK_OFF;
    else if (ssl_game_state.directKick ())
      curr_plan_index_ = DIRECT_KICK;
    else if (ssl_game_state.indirectKick ())
      curr_plan_index_ = INDIRECT_KICK;
    else if (ssl_game_state.penaltyKick ())
      curr_plan_index_ = PENALTY_KICK;
    else if (ssl_game_state.gameOn ())
    {
      curr_plan_index_ = BACK_AND_FORTH;
      if (prev_plan_index_ != BACK_AND_FORTH)
        initBackAndForth ();
    }
  }
}

void
SSLGamePlanner::goToPoint (uint8_t id, double orientation, geometry_msgs::Point32 point)
{
  target_poses[id].x = point.x;
  target_poses[id].y = point.y;
  target_poses[id].theta = orientation;

  ssl_msgs::RobotPoseControl robot_pose_control;
  robot_pose_control.id = id;
  robot_pose_control.team = team_;
  robot_pose_control.pose.x = target_poses[id].x;
  robot_pose_control.pose.y = target_poses[id].y;
  robot_pose_control.pose.theta = target_poses[id].theta;
  pose_control.pose.push_back (robot_pose_control);
}

void
SSLGamePlanner::attack ()
{
  //individual goal assignment
  pose_control.pose.clear ();

  goToBall (0, ssl::math::PI / 2.0);

  pub_pose_control.publish (pose_control);
}

geometry_msgs::Point32
SSLGamePlanner::getCurrentPosition (uint8_t team, uint8_t id)
{
  geometry_msgs::Point32 curr_position;
  if (team_ == ssl::BLUE)
  {
    curr_position.x = global_state_.blue_team[id].pose.x;
    curr_position.y = global_state_.blue_team[id].pose.y;
    curr_position.z = 0.0;
  }
  else
  {
    curr_position.x = global_state_.yellow_team[id].pose.x;
    curr_position.y = global_state_.yellow_team[id].pose.y;
    curr_position.z = 0.0;
  }

  return curr_position;
}

geometry_msgs::Point32
SSLGamePlanner::getCurrentPosition (uint8_t id)
{
  geometry_msgs::Point32 curr_position;
  curr_position.x = team_global_state[id].pose.x;
  curr_position.y = team_global_state[id].pose.y;
  curr_position.z = 0.0;
  return curr_position;
}

double
SSLGamePlanner::getSquaredDistance (geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
  double delta_x = p1.x - p2.x;
  double delta_y = p1.y - p2.y;
  //  double delta_z = p1.z - p2.z;

  return delta_x * delta_x + delta_y * delta_y/* + delta_z * delta_z*/;
}

void
SSLGamePlanner::goToBall (uint8_t id, double orientation)
{
  if (global_st.balls.size () > 0)
  {
    ssl_msgs::RobotPoseControl robot_pose_control;
    //    geometry_msgs::Point32 target_position;
    geometry_msgs::Point32 current_position = getCurrentPosition (id);
    geometry_msgs::Point32 target_position;
    target_position.x = target_poses[id].x;
    target_position.y = target_poses[id].y;
    geometry_msgs::Point32 ball_position = global_st.balls[0].position;
    double distance = sqrt (getSquaredDistance (current_position, target_position));
    double ang = orientation + ssl::math::PI;

    if (distance > BALL_PRE_POSE_OFFSET)
    {
      target_poses[id].x = ball_position.x + BALL_PRE_POSE_OFFSET * cos (ang);
      target_poses[id].y = ball_position.y + BALL_PRE_POSE_OFFSET * sin (ang);
      target_poses[id].theta = orientation;
      robot_pose_control.flag_ball_obs = true;
    }
    else// if(distance > BALL_FETCH_OFFSET)
    {
      target_poses[id].x = ball_position.x + BALL_FETCH_OFFSET * cos (ang);
      target_poses[id].y = ball_position.y + BALL_FETCH_OFFSET * sin (ang);
      target_poses[id].theta = orientation;
      robot_pose_control.flag_ball_obs = false;
    }
    robot_pose_control.pose.x = target_poses[id].x;
    robot_pose_control.pose.y = target_poses[id].y;
    robot_pose_control.pose.theta = target_poses[id].theta;
    pose_control.pose.push_back (robot_pose_control);
  }
}

void
SSLGamePlanner::goToBall (uint8_t id, uint8_t face_to_team_mate_id)
{
}

bool
SSLGamePlanner::send ()
{
  pub_pose_control.publish (pose_control);
  return true;
}

bool
SSLGamePlanner::isPlanChanged ()
{
  if (curr_plan_index_ != prev_plan_index_)
    return true;
  else
    return false;
}

bool
SSLGamePlanner::isPoseControlUpdated ()
{
  if (pose_control.pose.size () > 0)
    return true;
  else
    return false;
}

void
SSLGamePlanner::run ()
{
  init ();
  //  runPlan (curr_plan_index_);
  //ros::Rate r(1000/ssl::config::TIME_STEP);
  while (nh_.ok ())
  {
    //if global_state is updated and path_planner is connected
    // then make a plan
    if (update ())
    {
      evaluate ();
      std::cout << "**" << std::endl;
      std::cout << "allowed_near_ball? " << ssl_game_state.allowedNearBall () << std::endl;
      std::cout << "can move?          " << ssl_game_state.canMove () << std::endl;
      std::cout << "can kick ball?     " << ssl_game_state.canKickBall () << std::endl;
      std::cout << "--" << std::endl;
      runPlan (curr_plan_index_);
      if (isPlanChanged () || isPoseControlUpdated ())
      {
        send ();
        prev_plan_index_ = curr_plan_index_;
      }
    }

    ros::spinOnce ();
    //r.sleep();
  }
}
