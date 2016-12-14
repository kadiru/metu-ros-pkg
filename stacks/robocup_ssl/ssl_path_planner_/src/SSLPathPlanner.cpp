/*  Copyright (C) 2011, Kadir Firat Uyanik 
 *	Middle East Technical University, Kovan Research Lab
 *	kadir@ceng.metu.edu.tr
 *
 *	http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  SSLPathPlanner.cpp is part of ssl_path_planner.
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

#include "../include/ssl_path_planner/SSLPathPlanner.h"

void
SSLPathPlanner::globalStateRcvd (const ssl_msgs::GlobalState::ConstPtr &global_msg)
{
  if (!global_st_rcvd)
    global_st_rcvd = true;
  global_state = *global_msg;

  if (team_ == ssl::BLUE)
  {
    for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
      team_state_[i] = global_state.blue_team[i];
  }
  else
  {
    for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
      team_state_[i] = global_state.yellow_team[i];
  }

  updateObstacles ();
}

void
SSLPathPlanner::odomStateRcvd (const ssl_msgs::AllOdomStates::ConstPtr &odoms_msg)
{
  if (!odom_rcvd)
    odom_rcvd = true;
  all_odoms_ = *odoms_msg;
}

Obstacle
SSLPathPlanner::getGoalAsObstacle2 (ssl::FIELD_SIDES left_or_right)
{
  Obstacle obstacle;
  obstacle.type = ssl::GOAL;
  Rectancle_2 rectangle_upper;
  Rectancle_2 rectangle_bottom;

  double pad = ssl::config::ROBOT_BOUNDING_RADIUS;

  if (left_or_right == ssl::LEFT)
  {
    rectangle_upper = Rectancle_2 (Point_2 (-(3.050 + pad), 0.35 + pad), Point_2 (-(3.025 + pad), 0.37 + pad));
    rectangle_bottom = Rectancle_2 (Point_2 (-(3.050 + pad), -(0.37 + pad)), Point_2 (-3.025, -(0.35 + pad)));
  }
  else
  {
    rectangle_upper = Rectancle_2 (Point_2 (3.025 + pad, 0.35 + pad), Point_2 (3.050 + pad, 0.37 + pad));
    rectangle_bottom = Rectancle_2 (Point_2 (3.025 + pad, -(0.37 + pad)), Point_2 (3.050 + pad, -(0.35 + pad)));
  }
  obstacle.rectangles.push_back (rectangle_bottom);
  obstacle.rectangles.push_back (rectangle_upper);

  return obstacle;
}

Obstacle
SSLPathPlanner::getGoalAsObstacle (ssl::FIELD_SIDES left_or_right)
{
  Obstacle obstacle;
  obstacle.type = ssl::GOAL;
  Rectancle_2 rectangle_upper;
  Rectancle_2 rectangle_bottom;

  if (left_or_right == ssl::LEFT)
  {
    rectangle_upper = Rectancle_2 (Point_2 (-3.050, 0.35), Point_2 (-3.025, 0.37));
    rectangle_bottom = Rectancle_2 (Point_2 (-3.050, -0.37), Point_2 (-3.025, -0.35));
  }
  else
  {
    rectangle_upper = Rectancle_2 (Point_2 (3.025, 0.35), Point_2 (3.050, 0.37));
    rectangle_bottom = Rectancle_2 (Point_2 (3.025, -0.37), Point_2 (3.050, -0.35));
  }

  Point upper_points[] = {rectangle_upper.vertex (0), rectangle_upper.vertex (1), rectangle_upper.vertex (2),
                          rectangle_upper.vertex (3)};

  Point lower_points[] = {rectangle_bottom.vertex (0), rectangle_bottom.vertex (1), rectangle_bottom.vertex (2),
                          rectangle_bottom.vertex (3)};

  //  Point upper_points[] = {Point (ssl::math::sign ((int8_t)left_or_right) * 3.050, 0.35),
  //                          Point (ssl::math::sign ((int8_t)left_or_right) * 3.050, 0.37),
  //                          Point (ssl::math::sign ((int8_t)left_or_right) * 3.025, 0.37),
  //                          Point (ssl::math::sign ((int8_t)left_or_right) * 3.025, 0.35)};
  //
  //  Point lower_points[] = {Point (ssl::math::sign ((int8_t)left_or_right) * 3.025, -0.35),
  //                          Point (ssl::math::sign ((int8_t)left_or_right) * 3.025, -0.37),
  //                          Point (ssl::math::sign ((int8_t)left_or_right) * 3.050, -0.35),
  //                          Point (ssl::math::sign ((int8_t)left_or_right) * 3.050, -0.37)};

  obstacle.polygons.push_back (Polygon_2 (upper_points, upper_points + 4));
  obstacle.polygons.push_back (Polygon_2 (lower_points, lower_points + 4));

  return obstacle;
}

Obstacle
SSLPathPlanner::getBallAsObstacle ()
{
  Obstacle obstacle;
  obstacle.type = ssl::BALL;
  obstacle.circles.push_back (Circle_2 (Point_2 (global_state.balls[0].position.x, global_state.balls[0].position.y),
                                        ssl::config::BALL_RADIUS));
  return obstacle;
}

void
SSLPathPlanner::updateObstacles ()
{
  obstacles_.clear ();
  //  for(uint32_t i=0;i<obstacles_.size();i++)
  //  {
  //    obstacles_[i].circles.clear();
  //    obstacles_[i].polygons.clear();
  //  }

  //for blue and yellow teams
  for (unsigned int i = 0; i < ssl::config::TEAM_CAPACITY; i++)
  {
    if (global_state.blue_team[i].state != ssl::OUT_OF_FOV)
      obstacles_.push_back (getRobotAsObstacle (ssl::BLUE, i));

    if (global_state.yellow_team[i].state != ssl::OUT_OF_FOV)
      obstacles_.push_back (getRobotAsObstacle (ssl::YELLOW, i));
  }

  //for goals
  obstacles_.push_back (getGoalAsObstacle2 (ssl::LEFT));
  obstacles_.push_back (getGoalAsObstacle2 (ssl::RIGHT));

  //most confident ball
  if (global_state.balls.size () > 0)
    obstacles_.push_back (getBallAsObstacle ());

  //  std::cout << "n_obstacles: " << obstacles_.size () << std::endl;
  //  for (uint32_t i = 0; i < obstacles_.size (); i++)
  //  {
  //    for (uint8_t j = 0; j < obstacles_[i].circles.size (); j++)
  //      std::cout << obstacles_[i].circles[j] << std::endl;
  //
  //    for (uint8_t j = 0; j < obstacles_[i].polygons.size (); j++)
  //      std::cout << obstacles_[i].polygons[j] << std::endl;
  //  }
}

Obstacle
SSLPathPlanner::getRobotAsObstacle (uint8_t team, uint8_t id, const geometry_msgs::Vector3& vel,
                                    const geometry_msgs::Vector3& acc)
{
  Obstacle obstacle = getRobotAsObstacle (team, id);
  //assuming linear velocity in one control cycle which is about 20ms.
  tf::Vector3 x_curr;
  tf::vector3MsgToTF (getRobotPosition (team, id), x_curr);

  tf::Vector3 v_curr;
  tf::vector3MsgToTF (vel, v_curr);

  tf::Vector3 acc_cmd;
  tf::vector3MsgToTF (acc, acc_cmd);

  tf::Vector3 x_next = x_curr + v_curr * ssl::config::TIME_STEP_SEC + 0.5 * acc_cmd * ssl::config::TIME_STEP_SEC
      * ssl::config::TIME_STEP_SEC;

  obstacle.circles.push_back (Circle_2 (Point_2 (x_next.x (), x_next.y ()), ssl::config::ROBOT_BOUNDING_RADIUS));

  //TODO fill in between these circles with a rectangular polygon
  // skipping this for now since, a robot in 20ms can move at most 6cm while moving with 3m/s
  return obstacle;
}

//assumes the robot is statical
Obstacle
SSLPathPlanner::getRobotAsObstacle (uint8_t team, uint8_t id)
{
  ssl_msgs::GlobalRobotState robot_state;

  Obstacle obstacle;
  obstacle.type = ssl::ROBOT;
  obstacle.id = id;

  if (team == ssl::BLUE)
  {
    robot_state = global_state.blue_team[id];
    obstacle.team = ssl::BLUE;
  }
  else
  {
    robot_state = global_state.yellow_team[id];
    obstacle.team = ssl::YELLOW;
  }

  if (robot_state.state != ssl::OUT_OF_FOV)
  {
    Point_2 center = Point_2 (robot_state.pose.x, robot_state.pose.y);
    Circle_2 obstacle_circle = Circle_2 (center, ssl::config::ROBOT_BOUNDING_RADIUS);
    obstacle.circles.push_back (obstacle_circle);
  }
  return obstacle;
}

void
SSLPathPlanner::markWaypoint (geometry_msgs::Point point, int32_t type, uint32_t robot_index)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/field";
  marker.header.stamp = ros::Time ();
  //marker.ns = "my_namespace";
  marker.id = type + 5 * robot_index;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  if (type == 0)//prev
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else if (type == 1)//nearest
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  else if (type == 2)//next
  {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  else if (type == 3)//current
  {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.scale.x = 0.09;
    marker.scale.y = 0.09;
    marker.scale.z = 0.15;
    marker.type = visualization_msgs::Marker::CYLINDER;
  }

  vis_pub.publish (marker);
}

geometry_msgs::Vector3
SSLPathPlanner::getRobotPosition (uint8_t team, uint32_t id)
{
  geometry_msgs::Vector3 v;
  if (team == ssl::BLUE)
  {
    v.x = global_state.blue_team[id].pose.x;
    v.y = global_state.blue_team[id].pose.y;
    v.z = 0.0;
  }
  else
  {
    v.x = global_state.yellow_team[id].pose.x;
    v.y = global_state.yellow_team[id].pose.y;
    v.z = 0.0;
  }
  return v;
}

SSLPathPlanner::SSLPathPlanner (ros::NodeHandle& n, const int& team) :
  nh (n)
{
  team_ = team;

  std::string team_name;
  if (team_ == 0)
    team_name.assign (ssl::naming::entity::BLUE_TEAM);
  else
    team_name.assign (ssl::naming::entity::YELLOW_TEAM);

  std::string global_state_topic_name = "/" + team_name + "/" + ssl::naming::topic::ESTIMATED_GLOBAL_ST;
  std::string robot_path_plan_topic_name = "/" + team_name;
  std::string pose_control_topic_name = team_name + "_" + ssl::naming::POSE_CTRL_TOPIC_NAME;

  sub_global_state = nh.subscribe (global_state_topic_name, 10, &SSLPathPlanner::globalStateRcvd, this);
  sub_all_odoms_ = nh.subscribe (team_name + "/odoms", 10, &SSLPathPlanner::odomStateRcvd, this);
  sub_pose_control = nh.subscribe (pose_control_topic_name, 10, &SSLPathPlanner::poseControlRcvd, this);
  vis_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 0);

  path_plans_.resize (ssl::config::TEAM_CAPACITY);
  pub_robot_path_plans_.resize (ssl::config::TEAM_CAPACITY);
  pub_vel_cmds.resize (ssl::config::TEAM_CAPACITY);
  is_plan_done.resize (ssl::config::TEAM_CAPACITY, false);

  for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
  {
    pub_vel_cmds[i] = nh.advertise<geometry_msgs::Vector3> (ssl::naming::createRobotName (i, team_) + "/actuation", 10);
  }

  for (uint8_t i = 0; i < ssl::config::TEAM_CAPACITY; i++)
  {
    std::string topic_name = robot_path_plan_topic_name;
    std::stringstream s;
    s << (int)i;
    topic_name.append (s.str ());
    topic_name.append ("/path");
    pub_robot_path_plans_[i] = nh.advertise<nav_msgs::Path> (topic_name, 10);
  }

  for (unsigned int i = 0; i < ssl::config::TEAM_CAPACITY; i++)
    update_pose_controls[i] = false;

  global_st_rcvd = false;
  pose_ctrl_rcvd = false;

  tmp_robot_id_queried = 0;

  pose_control.pose.resize (ssl::config::TEAM_CAPACITY);
  team_state_.resize (ssl::config::TEAM_CAPACITY);
  flag_ball_obs.resize (ssl::config::TEAM_CAPACITY, false);
  next_target_poses.resize (ssl::config::TEAM_CAPACITY);
}

SSLPathPlanner::~SSLPathPlanner ()
{
  ros::shutdown ();
}

void
SSLPathPlanner::poseControlRcvd (const ssl_msgs::PoseControl::ConstPtr pose_msg)
{
  if (!pose_ctrl_rcvd)
    pose_ctrl_rcvd = true;

  for (unsigned int i = 0; i < pose_msg->pose.size (); i++)
  {
    const int id = pose_msg->pose[i].id;
    if (robots_control_type[id] == ssl::AUTONOMOUS)
    {
//      if(!(pose_control.pose[id].pose.x == pose_msg->pose[i].pose.x &&
//          pose_control.pose[id].pose.y == pose_msg->pose[i].pose.y &&
//          pose_control.pose[id].pose.theta == pose_msg->pose[i].pose.theta))
//      {
        pose_control.pose[id] = pose_msg->pose[i];
        flag_ball_obs[id] = pose_control.pose[id].flag_ball_obs;
        std::cout << pose_control.pose[id] << std::endl;
        update_pose_controls[id] = true;
        is_plan_done[id] = false;
//      }
    }
  }
}

bool
SSLPathPlanner::doesIntersectObstacles (const int& id, const Point_2& start, const Point_2& goal)
{
  Segment_2 direct_path (start, goal);

  for (unsigned int i = 0; i < obstacles_.size (); i++)
  {
    //check for goal
    if (obstacles_[i].type == (int)ssl::GOAL)
    {
      //      Obstacle tmp_obstacle = obstacles_[i];
      for (uint8_t j = 0; j < obstacles_[i].rectangles.size (); j++)
      {
        if (CGAL::do_intersect (obstacles_[i].rectangles[j], direct_path))
          return true;
      }
    }
    else if (obstacles_[i].type == (int)ssl::BALL && flag_ball_obs[id])
    {
      double squared_central_distance = CGAL::to_double (CGAL::squared_distance (obstacles_[i].circles[0].center (),
                                                                                 direct_path));
      if (squared_central_distance < pow (1.1 * (ssl::config::ROBOT_BOUNDING_RADIUS + ssl::config::BALL_RADIUS), 2.0))
        return true;
    }
    else
    {
      //check for other robots
      if (!(obstacles_[i].team == team_ && obstacles_[i].id == id))
      {
        for (unsigned int c = 0; c < obstacles_[i].circles.size (); c++)
        {
          double squared_central_distance =
              CGAL::to_double (CGAL::squared_distance (obstacles_[i].circles[c].center (), direct_path));

          if (squared_central_distance < pow (2.1 * ssl::config::ROBOT_BOUNDING_RADIUS, 2.0))
            return true;
        }
      }
    }
  }
  return false;
}

bool
SSLPathPlanner::doesIntersectObstacles (const int& id, const Point_2& center)
{
  Obstacle obstacle;
  obstacle.team = team_;
  obstacle.id = id;
  obstacle.circles.push_back (Circle_2 (center, ssl::config::ROBOT_BOUNDING_RADIUS));
  obstacle.type = ssl::ROBOT;

  for (unsigned int i = 0; i < obstacles_.size (); i++)
  {
    //check for goal
    if (obstacles_[i].type == (int)ssl::GOAL)
    {
      //      Obstacle tmp_obstacle = obstacles_[i];
      for (uint8_t j = 0; j < obstacles_[i].rectangles.size (); j++)
      {
        for (uint8_t l = 0; l < obstacle.circles.size (); l++)
        {
          if (obstacles_[i].rectangles[j].has_on_bounded_side (obstacle.circles[l].center ()))
            return true;
        }
      }
    }
    else if (obstacles_[i].type == (int)ssl::BALL && flag_ball_obs[id])
    {
      for (uint8_t l = 0; l < obstacle.circles.size (); l++)
      {
        double squared_central_distance = CGAL::to_double (CGAL::squared_distance (obstacles_[i].circles[0].center (),
                                                                                   obstacle.circles[l].center ()));
        if (squared_central_distance < pow (1.1 * (ssl::config::ROBOT_BOUNDING_RADIUS + ssl::config::BALL_RADIUS), 2.0))
          return true;
      }
    }
    else
    {
      //check for other robots
      if (!(obstacles_[i].team == team_ && obstacles_[i].id == id))
      {
        for (unsigned int c = 0; c < obstacles_[i].circles.size (); c++)
        {
          for (uint8_t l = 0; l < obstacle.circles.size (); l++)
          {
            double squared_central_distance =
                CGAL::to_double (CGAL::squared_distance (obstacles_[i].circles[c].center (),
                                                         obstacle.circles[l].center ()));
            //        std::cout<<"squared_distance  "<<squared_central_distance<<std::endl;
            //        std::cout<<"critical_distance "<<pow(2*ssl::config::ROBOT_BOUNDING_RADIUS,2.0)<<std::endl;
            if (squared_central_distance < pow (2.1 * ssl::config::ROBOT_BOUNDING_RADIUS, 2.0))
              return true;
          }
        }
      }
    }
  }
  return false;
}

bool
SSLPathPlanner::isStateValid (const State* state)
{
  const RealVectorStateManifold::StateType *pos = state->as<RealVectorStateManifold::StateType> ();

  Point_2 center = Point_2 (pos->values[0], pos->values[1]);

  return !doesIntersectObstacles (tmp_robot_id_queried, center);
  //  return true;
}

double
SSLPathPlanner::getSquaredDistance (const State* state_1, const ScopedState<RealVectorStateManifold>& state_2)
{
  const RealVectorStateManifold::StateType *position_1 = state_1->as<RealVectorStateManifold::StateType> ();

  double delta_x = position_1->values[0] - state_2->values[0];
  double delta_y = position_1->values[1] - state_2->values[1];

  return delta_x * delta_x + delta_y * delta_y;
}

double
SSLPathPlanner::getSquaredDistance (const State* state_1, const State* state_2)
{
  const RealVectorStateManifold::StateType *position_1 = state_1->as<RealVectorStateManifold::StateType> ();

  const RealVectorStateManifold::StateType *position_2 = state_2->as<RealVectorStateManifold::StateType> ();

  double delta_x = position_1->values[0] - position_2->values[0];
  double delta_y = position_1->values[1] - position_2->values[1];

  return delta_x * delta_x + delta_y * delta_y;
}

int
SSLPathPlanner::getNearestOnwardStateIndex (const std::vector<State*>& states, const ScopedState<
    RealVectorStateManifold>& queried_state)
{
  unsigned int nearest_state_index = getNearestStateIndex (states, queried_state);

  //if it is not the goal state
  if (nearest_state_index < states.size () - 1)
  {
    //if the nearest state is already being passed return the onward state
    double squared_dist_later = getSquaredDistance (states[nearest_state_index + 1], queried_state);
    double squared_dist_between = getSquaredDistance (states[nearest_state_index], states[nearest_state_index + 1]);

    //if it is closer to reach later state from current state, no need to back up to go back the former state
    if (squared_dist_later < squared_dist_between)
      return nearest_state_index + 1;
    else
      //first visit the former state since it is on the way
      return nearest_state_index;
  }
  else
    return nearest_state_index;
}

int
SSLPathPlanner::getNearestOnwardStateIndex (const std::vector<State*>& states, const State* queried_state)
{
  int nearest_state_index = getNearestStateIndex (states, queried_state);
  if (nearest_state_index == -1)
    return -1;

  //if it is not the goal state
  if (nearest_state_index < states.size () - 1)
  {
    std::cout << states.size () << std::endl;
    std::cout << nearest_state_index << std::endl;
    //if the nearest state is already being passed return the onward state
    double squared_dist_later = getSquaredDistance (states[nearest_state_index + 1], queried_state);
    double squared_dist_between = getSquaredDistance (states[nearest_state_index], states[nearest_state_index + 1]);

    //if it is closer to reach later state from current state, no need to back up to go back the former state
    if (squared_dist_later < squared_dist_between)
      return nearest_state_index + 1;
    else
      //first visit the former state since it is on the way
      return nearest_state_index;
  }
  else
    return nearest_state_index;
}

int
SSLPathPlanner::getNearestStateIndex (const std::vector<State*>& states,
                                      const ScopedState<RealVectorStateManifold>& queried_state)
{
  if (states.size () == 0)
    return -1;

  double min_squared_distance = getSquaredDistance (states[0], queried_state);
  int nearest_state_index = 0;

  for (unsigned int i = 1; i < states.size (); i++)
  {
    double squared_distance = getSquaredDistance (states[i], queried_state);
    if (squared_distance < min_squared_distance)
    {
      min_squared_distance = squared_distance;
      nearest_state_index = i;
    }
  }
  return nearest_state_index;
}

int
SSLPathPlanner::getNearestStateIndex (const std::vector<State*>& states, const State* queried_state)
{
  if (states.size () == 0)
    return -1;

  double min_squared_distance = getSquaredDistance (states[0], queried_state);
  int nearest_state_index = 0;

  for (unsigned int i = 1; i < states.size (); i++)
  {
    double squared_distance = getSquaredDistance (states[i], queried_state);
    if (squared_distance < min_squared_distance)
    {
      min_squared_distance = squared_distance;
      nearest_state_index = i;
    }
  }
  return nearest_state_index;
}

void
SSLPathPlanner::getTrimmedPath (PathGeometric& trimmed_path, const std::vector<State*>& states, int trim_index)
{
  unsigned int offset = trimmed_path.states.size ();
  trimmed_path.states.resize (offset + states.size () - trim_index);
  for (unsigned int i = trim_index; i < states.size (); i++)
  {
    trimmed_path.states[i - trim_index + offset] = manifold->allocState ();
    manifold->copyState (trimmed_path.states[i - trim_index + offset], states[i]);
  }
}

double
SSLPathPlanner::getSquaredDistance (Point_2 p1, Point_2 p2)
{
  double delta_x = p1.x () - p2.x ();
  double delta_y = p1.y () - p2.y ();

  return delta_x * delta_x + delta_y * delta_y;
}

bool
SSLPathPlanner::isPlanDone (uint8_t id)
{
  Point_2 p_start (team_state_[id].pose.x, team_state_[id].pose.y);
  Point_2 p_goal (pose_control.pose[id].pose.x, pose_control.pose[id].pose.y);

  if (getSquaredDistance (p_start, p_goal) < POSE_REACHED_DIST)
  {
    is_plan_done[id] = true;
    return true;
  }
  else
    return false;
}

bool
SSLPathPlanner::doPlanForRobot (const int& id/*, bool& is_stop*/)
{
  //  is_send_plan = true;
  tmp_robot_id_queried = id;

  State* tmp_start = manifold->allocState ();
  tmp_start->as<RealVectorStateManifold::StateType> ()->values[0] = team_state_[id].pose.x;
  tmp_start->as<RealVectorStateManifold::StateType> ()->values[1] = team_state_[id].pose.y;

  PathGeometric direct_path (planner_setup->getSpaceInformation ());
  direct_path.states.push_back (manifold->allocState ());
  manifold->copyState (direct_path.states[0], tmp_start);

  State* tmp_goal = manifold->allocState ();

  tmp_goal->as<RealVectorStateManifold::StateType> ()->values[0] = pose_control.pose[id].pose.x;
  tmp_goal->as<RealVectorStateManifold::StateType> ()->values[1] = pose_control.pose[id].pose.y;
  //  std::cout<<pose_control.pose[id].pose.x<<"\t"<<pose_control.pose[id].pose.y<<std::endl;

  direct_path.states.push_back (manifold->allocState ());
  manifold->copyState (direct_path.states[1], tmp_goal);

  Point_2 p_start (team_state_[id].pose.x, team_state_[id].pose.y);
  Point_2 p_goal (pose_control.pose[id].pose.x, pose_control.pose[id].pose.y);

  //too close to the target point, no need for planning, just take it as a next_target_pose
  if (sqrt (getSquaredDistance (p_start, p_goal)) < VERY_CRITICAL_DIST)
  {
    next_target_poses[id].x = pose_control.pose[id].pose.x;
    next_target_poses[id].y = pose_control.pose[id].pose.y;
    next_target_poses[id].theta = pose_control.pose[id].pose.theta;
    return true;
  }

  //  manifold->printState (direct_path.states[0], std::cout);
  //  manifold->printState (direct_path.states[1], std::cout);

  //direct path is available, no need for planning
  //  if (direct_path.check ())
  if (!doesIntersectObstacles (id, p_start, p_goal))
  {
    //    std::cout << "direct path is AVAILABLE for robot " << id << std::endl;

    if (solution_data_for_robots[id].size () > direct_path.states.size ())
    {
      for (uint32_t i = direct_path.states.size (); i < solution_data_for_robots[id].size (); i++)
        manifold->freeState (solution_data_for_robots[id][i]);
      solution_data_for_robots[id].resize (direct_path.states.size ());
    }
    else if (solution_data_for_robots[id].size () < direct_path.states.size ())
    {
      for (uint32_t i = solution_data_for_robots[id].size (); i < direct_path.states.size (); i++)
        solution_data_for_robots[id].push_back (manifold->allocState ());
    }
    for (uint32_t i = 0; i < direct_path.states.size (); i++)
      manifold->copyState (solution_data_for_robots[id][i], direct_path.states[i]);

    manifold->freeState (tmp_start);
    manifold->freeState (tmp_goal);

    next_target_poses[id].x = pose_control.pose[id].pose.x;
    next_target_poses[id].y = pose_control.pose[id].pose.y;
    next_target_poses[id].theta = pose_control.pose[id].pose.theta;
    //    std::cout<<next_target_poses[id].x<<"\t"<<next_target_poses[id].y<<std::endl;

    return true;
  }
  manifold->freeState (tmp_start);
  manifold->freeState (tmp_goal);
  //  std::cout << "direct path is NOT AVAILABLE for robot " << id << ", doing planning" << std::endl;
  //direct path is not available, DO PLAN if the earlier plan is invalidated!

  //earlier plan is still valid
  if (checkPlanForRobot (id))
    return true;
  else if (is_plan_done[id])
    return false;

  //earlier plan is not valid anymore, replan
  planner_setup->clear ();
  planner_setup->clearStartStates ();

  ScopedState<RealVectorStateManifold> start_state (manifold);
  ScopedState<RealVectorStateManifold> goal_state (manifold);

  start_state->values[0] = team_state_[id].pose.x;
  start_state->values[1] = team_state_[id].pose.y;

  goal_state->values[0] = pose_control.pose[id].pose.x;
  goal_state->values[1] = pose_control.pose[id].pose.y;

  planner_setup->setStartAndGoalStates (start_state, goal_state);
  planner_setup->getProblemDefinition ()->fixInvalidInputStates (ssl::config::ROBOT_BOUNDING_RADIUS / 2.0,
                                                                 ssl::config::ROBOT_BOUNDING_RADIUS / 2.0, 100);
  bool solved = planner_setup->solve (0.100);//100msec

  if (solved)
  {
    planner_setup->simplifySolution ();

    PathGeometric* path;
    path = &planner_setup->getSolutionPath ();

    //    PathSimplifier p (planner_setup->getSpaceInformation ());
    //    p.reduceVertices (*path, 1000);

    if (solution_data_for_robots[id].size () < path->states.size ())
    {
      for (unsigned int i = solution_data_for_robots[id].size (); i < path->states.size (); i++)
        solution_data_for_robots[id].push_back (manifold->allocState ());
    }
    else if (solution_data_for_robots[id].size () > path->states.size ())
    {
      for (unsigned int i = path->states.size (); i < solution_data_for_robots[id].size (); i++)
        manifold->freeState (solution_data_for_robots[id][i]);
      //drop last elements that are already being freed
      solution_data_for_robots[id].resize (path->states.size ());
    }

    for (unsigned int i = 0; i < path->states.size (); i++)
      manifold->copyState (solution_data_for_robots[id][i], path->states[i]);

    //leader-follower approach based segment enlargement
    //pick next location
    Point_2 curr_point (path->states[0]->as<RealVectorStateManifold::StateType> ()->values[0], path->states[0]->as<
        RealVectorStateManifold::StateType> ()->values[1]);

    for (uint32_t i = 1; i < path->states.size (); i++)
    {
      Point_2 next_point (path->states[i]->as<RealVectorStateManifold::StateType> ()->values[0], path->states[i]->as<
          RealVectorStateManifold::StateType> ()->values[1]);
      if (!doesIntersectObstacles (id, curr_point, next_point))
      {
        next_target_poses[id].x = path->states[i]->as<RealVectorStateManifold::StateType> ()->values[0];
        next_target_poses[id].y = path->states[i]->as<RealVectorStateManifold::StateType> ()->values[1];
        next_target_poses[id].theta = pose_control.pose[id].pose.theta;
      }
      else
        break;
    }
    //    next_target_poses[id].x = path->states[1]->as<RealVectorStateManifold::StateType> ()->values[0];
    //    next_target_poses[id].y = path->states[1]->as<RealVectorStateManifold::StateType> ()->values[1];

    //    planner_data_for_robots[id].clear ();
    //    planner_data_for_robots[id] = planner_setup->getPlannerData ();
  }
  return solved;
}

void
SSLPathPlanner::runRobot (const int& id)
{
  geometry_msgs::Vector3 v;
  if (!isPlanDone (id))
  {
    if (doPlanForRobot (id/*, is_stop*/))
    {
      sendPlanForRobot (id);
      v = exePlanForRobot (id);
      pub_vel_cmds[id].publish (v);
      return;
    }
  }

  v.x = 0;
  v.y = 0;
  double curr_theta = team_state_[id].pose.theta;
  double goal_theta = pose_control.pose[id].pose.theta;
  //  std::cout<<goal_theta<<std::endl;

  double angular_diff = getAngularDifference (curr_theta, goal_theta);
  //  std::cout<<"angular_diff: "<<angular_diff/ssl::math::PI*180.0<<std::endl;
  if (fabs (angular_diff) > ssl::math::PI / 2.0)
    v.z = -MAX_ANGULAR_VEL * ssl::math::sign (angular_diff);
  else
    v.z = -MAX_ANGULAR_VEL * (angular_diff / (ssl::math::PI / 2.0));

  pub_vel_cmds[id].publish (v);
}

void
SSLPathPlanner::initRobot (const int& id)
{
  //  bool is_stop = false;
  //  doPlanForRobot (id/*, is_stop*/);
  //  exePlanForRobot (id);
}

bool
SSLPathPlanner::checkPlanForRobot (const int& id)
{
  tmp_robot_id_queried = id;
  //if the robot is not currently supervised and a new pose message doesn't exist,
  //check validity of the previously obtained path and return true
  if (update_pose_controls[id])
  {
    update_pose_controls[id] = false;
    return false;
  }
  else //check validity of the current plan
  {
    //get nearest state from the tree that is onward.
    //create new PathGeometric that is trimmed version of the actual path
    //return the validity of the path

    geometry_msgs::Pose2D current_pose;
    if (team_ == 0)
      current_pose = global_state.blue_team[id].pose;
    else
      current_pose = global_state.yellow_team[id].pose;
    State* queried_state = manifold->allocState ();
    //    ScopedState<RealVectorStateManifold> queried_state (manifold);
    queried_state->as<RealVectorStateManifold::StateType> ()->values[0] = current_pose.x;
    queried_state->as<RealVectorStateManifold::StateType> ()->values[1] = current_pose.y;
    //    queried_state->values[0] = current_pose.x;
    //    queried_state->values[1] = current_pose.y;

    //    std::cout << "current state:" << std::endl;
    //    queried_state.print (std::cout);
    //
    //    std::cout << "solution states:" << std::endl;
    //    for(unsigned int i=0;i<solution_data_for_robots[id].size();i++)
    //      manifold->printState(solution_data_for_robots[id][i],std::cout);

    int nearest_onward_state_index = getNearestOnwardStateIndex (solution_data_for_robots[id], queried_state);
    if (nearest_onward_state_index == -1)
    {
      is_plan_done[id]=false;
      return false;
    }
    //    std::cout << "nearest state index:" << nearest_onward_state_index << std::endl;
    //trim the path

    PathGeometric trimmed_path (planner_setup->getSpaceInformation ());
    trimmed_path.states.push_back (manifold->allocState ());
    manifold->copyState (trimmed_path.states[0], queried_state);
    getTrimmedPath (trimmed_path, solution_data_for_robots[id], nearest_onward_state_index);
    //    std::cout << "trimmed path" << std::endl;
    //    trimmed_path.print (std::cout);
    //    std::cout<<"trimmed   dist: "<<trimmed_path.length()<<std::endl;
    //    std::cout<<"euclidean dist: "<<sqrt(getSquaredDistance(solution_data_for_robots[id].back(), queried_state))<<std::endl;
    bool is_path_valid = trimmed_path.check ();
    if (!is_path_valid)
    {
      std::cout << "path is not valid anymore, going to replan!" << std::endl;
    }
    else
    {
      const double euclidean_dis_to_goal = sqrt (getSquaredDistance (solution_data_for_robots[id].back (),
                                                                     queried_state));
      //path is valid but check it's smoothness for distance larger than 1m
      if (euclidean_dis_to_goal > 2.0 && trimmed_path.length () > 1.2 * euclidean_dis_to_goal)
      {
        manifold->freeState (queried_state);
        return false;
      }
    }
    manifold->freeState (queried_state);
    return is_path_valid;
  }
  return false;
}

geometry_msgs::Point
SSLPathPlanner::getCurrentPosition (uint8_t id)
{
  //  return all_odoms_.all_odoms[id].pose.pose.position;
  geometry_msgs::Point p;
  p.x = team_state_[id].pose.x;
  p.y = team_state_[id].pose.y;
  p.z = 0.0;

  return p;
}

geometry_msgs::Vector3
SSLPathPlanner::getCurrentVelocity (uint8_t id)
{
  return all_odoms_.all_odoms[id].twist.twist.linear;
}

double
SSLPathPlanner::getVelMagnitude (geometry_msgs::Vector3 vel)
{
  return sqrt (vel.x * vel.x + vel.y * vel.y);
}

double
SSLPathPlanner::getVelAngle (geometry_msgs::Vector3 vel)
{
  return atan2 (vel.y, vel.x);
}

double
SSLPathPlanner::getSquaredDistance (geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  double delta_x = p1.x - p2.x;
  double delta_y = p1.y - p2.y;
  double delta_z = p1.z - p2.z;

  return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
}

void
SSLPathPlanner::rotate (geometry_msgs::Vector3& vec, double angle)
{
  tf::Vector3 v;
  tf::vector3MsgToTF (vec, v);
  v = v.rotate (tf::Vector3 (0, 0, 1), angle);
  tf::vector3TFToMsg (v, vec);
}

double
SSLPathPlanner::getAngleBetween (const geometry_msgs::Point& src_point, const geometry_msgs::Point& dest_point)
{
  return atan2 (dest_point.y - src_point.y, dest_point.x - src_point.x);
}

double
SSLPathPlanner::getCurrentAngularVelocity (uint8_t id)
{
  return all_odoms_.all_odoms[id].twist.twist.angular.z;
}

double
SSLPathPlanner::getAngularDifference (double ref, double rel)
{
  double dot_product = cos (ref) * cos (rel) + sin (ref) * sin (rel);
  int sign = ssl::math::sign (sin (rel - ref));
  return acos (dot_product) * (-sign);
}

geometry_msgs::Vector3
SSLPathPlanner::exePlanForRobot (const int& id)
{
  geometry_msgs::Point curr_point = getCurrentPosition (id);

  geometry_msgs::Point next_point;
  next_point.x = next_target_poses[id].x;
  next_point.y = next_target_poses[id].y;
  next_point.z = 0.0;

  geometry_msgs::Point target_point;
  target_point.x = pose_control.pose[id].pose.x;
  target_point.y = pose_control.pose[id].pose.y;
  target_point.z = 0.0;

  double curr_theta = team_state_[id].pose.theta;
  //  std::cout<<curr_theta<<std::endl;
  double goal_theta = next_target_poses[id].theta;
  //  std::cout<<goal_theta<<std::endl;

  double angular_diff = getAngularDifference (curr_theta, goal_theta);
  //  std::cout<<"angular_diff: "<<angular_diff/ssl::math::PI*180.0<<std::endl;

  double angular_vel;
  if (fabs (angular_diff) > ssl::math::PI / 2.0)
    angular_vel = -MAX_ANGULAR_VEL * ssl::math::sign (angular_diff);
  else
    angular_vel = -MAX_ANGULAR_VEL * (angular_diff / (ssl::math::PI / 2.0));

  //  angular_vel = ssl::math::sign(angular_diff)*MAX_ANGULAR_VEL*(1-exp(-(fabs(angular_diff))));

  markWaypoint (next_point, 0, id);

  double dist = sqrt (getSquaredDistance (curr_point, next_point));
  double ang = getAngleBetween (curr_point, next_point);

  //  if(dist<VERY_CRITICAL_DIST)
  //  {
  //    geometry_msgs::Vector3 v;
  //    double v_mag = dist/CRITICAL_DISTANCE * CRITICAL_LINEAR_VEL;
  //    v.x = v_mag * cos(goal_theta);
  //    v.y = v_mag * sin(goal_theta);
  //    return v;
  //  }

  double vel_x = 0;
  double vel_y = 0;

  bool transit_pass = true;

  if (next_point.x == target_point.x && next_point.y == target_point.y && next_point.z == target_point.z)
  {
    transit_pass = false;
  }

  if (dist > CRITICAL_DISTANCE)
  {
    vel_x = MAX_LINEAR_VEL * cos (ang);
    vel_y = MAX_LINEAR_VEL * sin (ang);

    //now consider obstacles around
  }
  else
  {
    if (!transit_pass)
    {
      if (dist < VERY_CRITICAL_DIST)
      {
        vel_x = CRITICAL_LINEAR_VEL * (dist / VERY_CRITICAL_DIST) * cos (ang);
        vel_y = CRITICAL_LINEAR_VEL * (dist / VERY_CRITICAL_DIST) * sin (ang);
      }
      else
      {
        vel_x = MAX_LINEAR_VEL * (1 - exp (-dist * EXPONENT)) * cos (ang);
        vel_y = MAX_LINEAR_VEL * (1 - exp (-dist * EXPONENT)) * sin (ang);
      }
    }
    else
    {
      //TODO optimize this
      vel_x = INT_LINEAR_VEL * (1 - exp (-dist * EXPONENT)) * cos (ang);
      vel_y = INT_LINEAR_VEL * (1 - exp (-dist * EXPONENT)) * sin (ang);
    }
  }

  geometry_msgs::Vector3 v = getCurrentVelocity (id);
  //  double curr_vel = getVelMagnitude(v);
  //  double curr_vel_ang= getVelAngle(v);

  double des_acc_x = (vel_x - v.x) / ssl::config::TIME_STEP_SEC;
  double des_acc_y = (vel_y - v.y) / ssl::config::TIME_STEP_SEC;

  double des_acc = sqrt (des_acc_x * des_acc_x + des_acc_y * des_acc_y);
  double des_acc_ang = atan2 (des_acc_y, des_acc_x);
  if (des_acc > MAX_LINEAR_ACC)
    des_acc = MAX_LINEAR_ACC;

  des_acc_x = MAX_LINEAR_ACC * cos (des_acc_ang);
  des_acc_y = MAX_LINEAR_ACC * sin (des_acc_ang);

  v.x += des_acc_x * ssl::config::TIME_STEP_SEC * ACC_TUNING;
  v.y += des_acc_y * ssl::config::TIME_STEP_SEC * ACC_TUNING;

  double des_vel_ang = getVelAngle (v);
  double des_vel_mag = getVelMagnitude (v);

  if (des_vel_mag > MAX_LINEAR_VEL)
    des_vel_mag = MAX_LINEAR_VEL;

  v.x = des_vel_mag * cos (des_vel_ang);
  v.y = des_vel_mag * sin (des_vel_ang);

  tf::Vector3 tf_v;
  tf::vector3MsgToTF (v, tf_v);
  tf_v = tf_v.rotate (tf::Vector3 (0, 0, 1), -curr_theta);
  tf::vector3TFToMsg (tf_v, v);

  v.z = angular_vel;

  return v;
}

void
SSLPathPlanner::sendPlanForRobot (const int& id)
{
  if (pub_robot_path_plans_[id].getNumSubscribers () > 0)
  {
    unsigned int n_states = solution_data_for_robots[id].size ();
    if (n_states > 0)
    {
      path_plans_[id].poses.resize (n_states);
      for (unsigned int j = 0; j < n_states; j++)
      {
        const RealVectorStateManifold::StateType *pos = solution_data_for_robots[id][j]->as<
            RealVectorStateManifold::StateType> ();
        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = ssl::naming::frame::GLOBAL;
        //          pose.header.stamp = ros::Time::now ();

        pose.pose.position.x = pos->values[0];
        pose.pose.position.y = pos->values[1];
        pose.pose.position.z = 0.0;

        pose.pose.orientation = tf::createQuaternionMsgFromYaw (0);

        path_plans_[id].header.frame_id = ssl::naming::frame::GLOBAL;
        path_plans_[id].header.stamp = ros::Time::now ();
        path_plans_[id].poses[j] = pose;
      }
      pub_robot_path_plans_[id].publish (path_plans_[id]);
    }
  }
}

void
SSLPathPlanner::init ()
{
  robots_control_type[0] = ssl::AUTONOMOUS;
  robots_control_type[1] = ssl::AUTONOMOUS;
  robots_control_type[2] = ssl::AUTONOMOUS;
  robots_control_type[3] = ssl::AUTONOMOUS;
  robots_control_type[4] = ssl::AUTONOMOUS;

  manifold = StateManifoldPtr (new RealVectorStateManifold (2));

  RealVectorBounds bounds (2);
  bounds.setLow (0, -(ssl::config::FIELD_WIDTH / 2.0 + 0.5));
  bounds.setHigh (0, (ssl::config::FIELD_WIDTH / 2.0 + 0.5));
  bounds.setLow (1, -(ssl::config::FIELD_HEIGHT / 2.0 + 0.5));
  bounds.setHigh (1, (ssl::config::FIELD_HEIGHT / 2.0 + 0.5));

  manifold->as<RealVectorStateManifold> ()->setBounds (bounds);
  planner_setup = new SimpleSetup (manifold);
  planner_setup->setStateValidityChecker (boost::bind (&SSLPathPlanner::isStateValid, this, _1));

  pRRT* p_rrt = new pRRT (planner_setup->getSpaceInformation ());
  p_rrt->setGoalBias (0.8);
  p_rrt->setRange (0.08);
  p_rrt->setThreadCount (8);

  //  RRTConnect* rrt_connect = new RRTConnect (planner_setup->getSpaceInformation ());
  //  rrt_connect->setRange (0.08);
  //  planner_setup->setPlanner (PlannerPtr (rrt_connect));
  planner_setup->setPlanner (PlannerPtr (p_rrt));

  //wait for global_state to finish initialization procedure
  std::cout << "Path Planner is waiting for global_data" << std::endl;
  while (!global_st_rcvd && nh.ok ())
    ros::spinOnce ();

  //wait for pose_control to finish initialization procedure
  std::cout << "Path Planner is waiting for pose_control" << std::endl;
  while (!pose_ctrl_rcvd && nh.ok ())
    ros::spinOnce ();

  std::cout << "Path Planner is waiting for odom_control" << std::endl;
  while (!odom_rcvd && nh.ok ())
    ros::spinOnce ();

  //do initial plans for the robots according to the control configuration
  for (unsigned int i = 0; i < ssl::config::TEAM_CAPACITY; i++)
  {
    if (robots_control_type[i] != ssl::NO_CONTROL)
    {
      if (team_state_[i].state != ssl::OUT_OF_FOV)
        initRobot (i);
    }
  }
}

void
SSLPathPlanner::run ()
{
  //making plans for the robots that are pose_controlled
  while (nh.ok ())
  {
    for (unsigned int i = 0; i < ssl::config::TEAM_CAPACITY; i++)
    {
      if (robots_control_type[i] != ssl::NO_CONTROL)
      {
        if (team_state_[i].state != ssl::OUT_OF_FOV && !is_plan_done[i])
          runRobot (i);
      }
    }
    //    show ();
    ros::spinOnce ();
  }
}

