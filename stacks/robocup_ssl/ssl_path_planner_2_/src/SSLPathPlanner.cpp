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

SSLPathPlanner::SSLPathPlanner (ros::NodeHandle& n, const int& team) :
  nh (n)
{
  team_ = team;

  std::string pose_control_topic_name;
  std::string robot_path_plan_topic_name ("/");
  if (team_ == 0)
  {
    pose_control_topic_name.assign ("B");
    robot_path_plan_topic_name.append ("B");
  }
  else
  {
    pose_control_topic_name.assign ("Y");
    robot_path_plan_topic_name.append ("Y");
  }
  pose_control_topic_name.append ("_");
  pose_control_topic_name.append (ssl::naming::POSE_CTRL_TOPIC_NAME);

  sub_pose_control = nh.subscribe (pose_control_topic_name, 10, &SSLPathPlanner::poseControlRcvd, this);

  path_plans.resize (MAX_N_ROBOTS_PER_TEAM);
  pub_path_plans.resize (MAX_N_ROBOTS_PER_TEAM);
  for (uint8_t i = 0; i < MAX_N_ROBOTS_PER_TEAM; i++)
  {
    std::string topic_name = robot_path_plan_topic_name;
    std::stringstream s;
    s << (int)i;
    topic_name.append (s.str ());
    topic_name.append ("/motion_planner/path_plan");
    pub_path_plans[i] = nh.advertise<ssl_msgs::RobotPathPlan> (topic_name, 10);
  }
  pub_team_path_states = nh.advertise<ssl_msgs::TeamPoseStates> ("team_pose_states", 10);

  for (unsigned int i = 0; i < MAX_N_ROBOTS_PER_TEAM; i++)
    update_pose_controls[i] = false;

  pose_ctrl_rcvd = false;

  tmp_robot_id_queried = 0;

  pose_control.pose.resize (MAX_N_ROBOTS_PER_TEAM);
}

bool
SSLPathPlanner::update()
{
	updateObstacles ();
}


SSLPathPlanner::~SSLPathPlanner ()
{
  ros::shutdown ();
}

void
SSLPathPlanner::poseControlRcvd (const ssl_msgs::PoseControl::ConstPtr &pose_msg)
{
  std::cout << "hede" << std::endl;
  if (!pose_ctrl_rcvd)
    pose_ctrl_rcvd = true;
  for (unsigned int i = 0; i < pose_msg->pose.size (); i++)
  {
    const int id = pose_msg->pose[i].id;
    if (robots_control_type[id] == ssl::AUTONOMOUS)
    {
      pose_control.pose[id] = pose_msg->pose[i];
      std::cout << pose_control.pose[id] << std::endl;
      update_pose_controls[id] = true;
    }
    //TODO add SEMI_AUTONOMOUS case, where pose_control.pose[id] cannot be updated
    //if there is any ongoing supervised task
    //otherwise, as soon as supervised_control is received, after accomplishing that task
    //robot will not be able to turn back what is commanded by AI.
  }
}

void
SSLPathPlanner::manualPoseControlRcvd (const ssl_msgs::RobotPoseControl::ConstPtr &pose_msg)
{
  if (!pose_ctrl_rcvd)
    pose_ctrl_rcvd = true;

  const int id = pose_msg->id;
  if (robots_control_type[id] != ssl::AUTONOMOUS)
  {
    pose_control.pose[id] = *pose_msg;
    update_pose_controls[id] = true;
  }
}

void
SSLPathPlanner::updateObstacles ()
{
  obstacles_.clear ();

  //for blue team
  for (unsigned int i = 0; i < MAX_N_ROBOTS_PER_TEAM; i++)
  {
    Obstacle obstacle;
    if (global_state.blue_team[i].state != ssl::OUT_OF_FOV)
    {
      obstacle.type = ssl::ROBOT;
      obstacle.team = 0;
      obstacle.id = i;
      Point_2 center = Point_2 (global_state.blue_team[i].pose.x, global_state.blue_team[i].pose.y);
      Circle_2 obstacle_circle = Circle_2 (center, ssl::config::ROBOT_BOUNDING_RADIUS);
      obstacle.circles.push_back (obstacle_circle);
      obstacles_.push_back (obstacle);
    }
  }

  //for yellow team
  for (unsigned int i = 0; i < MAX_N_ROBOTS_PER_TEAM; i++)
  {
    Obstacle obstacle;
    if (global_state.yellow_team[i].state != ssl::OUT_OF_FOV)
    {
      obstacle.type = ssl::ROBOT;
      obstacle.team = 1;
      obstacle.id = i;
      Point_2 center = Point_2 (global_state.yellow_team[i].pose.x, global_state.yellow_team[i].pose.y);
      Circle_2 obstacle_circle = Circle_2 (center, ssl::config::ROBOT_BOUNDING_RADIUS);
      obstacle.circles.push_back (obstacle_circle);
      obstacles_.push_back (obstacle);
    }
  }
  //  std::cout<<"n_obstacles: "<<obstacles_.size()<<std::endl;
  //  for(unsigned int i=0;i<obstacles_.size();i++)
  //    std::cout<<obstacles_[i].circles[0]<<std::endl;
}

bool
SSLPathPlanner::doesIntersectObstacles (const int& id, const Point_2& center)
{
  //first get the robot's bounding circle
  Obstacle robot;
  robot.team = team_;
  robot.id = id;
  robot.circles.push_back (Circle_2 (center, ssl::config::ROBOT_BOUNDING_RADIUS));

  //  std::cout<<"team:"<<robot.team<<"id:"<<robot.id<<std::endl;
  //  for(unsigned int i=0;i<robot.circles.size();i++)
  //    std::cout<<robot.circles[i]<<std::endl;
  //  for(unsigned int i=0;i<robot.polygons.size();i++)
  //    std::cout<<robot.polygons[i]<<std::endl;
  //  std::cout<<"state:"<<center<<std::endl;

  //  std::cout<<"**"<<std::endl;
  for (unsigned int i = 0; i < obstacles_.size (); i++)
  {
    //    std::cout<<"++"<<std::endl;
    //    std::cout<<"team:"<<obstacles_[i].team<<"id:"<<obstacles_[i].id<<std::endl;
    //    for(unsigned int j=0;j<obstacles_[i].circles.size();j++)
    //      std::cout<<obstacles_[i].circles[j]<<std::endl;
    //    for(unsigned int j=0;j<obstacles_[i].polygons.size();j++)
    //      std::cout<<obstacles_[i].polygons[j]<<std::endl;

    if (!(obstacles_[i].team == team_ && obstacles_[i].id == id))
    {
      for (unsigned int c = 0; c < obstacles_[i].circles.size (); c++)
      {
        double squared_central_distance = CGAL::squared_distance (obstacles_[i].circles[c].center (),
                                                                  robot.circles[0].center ());
        //        std::cout<<"squared_distance  "<<squared_central_distance<<std::endl;
        //        std::cout<<"critical_distance "<<pow(2*ssl::config::ROBOT_BOUNDING_RADIUS,2.0)<<std::endl;
        if (squared_central_distance < pow (2 * ssl::config::ROBOT_BOUNDING_RADIUS, 2.0))
          return true;
      }
    }
    //    std::cout<<"--"<<std::endl;
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

bool
SSLPathPlanner::doPlanForRobot (const int& id)
{
  tmp_robot_id_queried = id;
  planner_setup->clear ();
  planner_setup->clearStartStates ();

  ScopedState<RealVectorStateManifold> start_state (manifold);
  ScopedState<RealVectorStateManifold> goal_state (manifold);

  if (team_ == 0)
  {
    start_state->values[0] = global_state.blue_team[id].pose.x;
    start_state->values[1] = global_state.blue_team[id].pose.y;
  }
  else
  {
    start_state->values[0] = global_state.yellow_team[id].pose.x;
    start_state->values[1] = global_state.yellow_team[id].pose.y;
  }
  goal_state->values[0] = pose_control.pose[id].pose.x;
  goal_state->values[1] = pose_control.pose[id].pose.y;

  planner_setup->setStartAndGoalStates (start_state, goal_state);
  bool solved = planner_setup->solve (0.10);//5msec

  if (solved)
  {
    planner_setup->simplifySolution ();

    PathGeometric* path;
    path = &planner_setup->getSolutionPath ();

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
    /*
     for(unsigned int i=0;i<solution_data_for_robots[id].size();i++)
     manifold->freeState (solution_data_for_robots[id][i]);

     solution_data_for_robots[id].clear();

     for (unsigned int i = 0; i < path->states.size (); i++)
     solution_data_for_robots[id].push_back (manifold->allocState ());
     */
    for (unsigned int i = 0; i < path->states.size (); i++)
      manifold->copyState (solution_data_for_robots[id][i], path->states[i]);

    planner_data_for_robots[id].clear ();
    planner_data_for_robots[id] = planner_setup->getPlannerData ();
  }
  return solved;
}

void
SSLPathPlanner::runRobot (const int& id)
{
  if (!checkPlanForRobot (id))
  {
    doPlanForRobot (id);
    showPlanForRobot (id);
    sendPlanForRobot (id);
  }

  exePlanForRobot (id);
}

void
SSLPathPlanner::initRobot (const int& id)
{
  doPlanForRobot (id);
  exePlanForRobot (id);
}

bool
SSLPathPlanner::checkPlanForRobot (const int& id)
{
  //TODO add direct path check here:
  // add current and goal states to a PathGeometric module
  // call valid() method, it valid update the path of this robot
  // and return true
  // if it is not valid go on with the validity check of the current path

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

bool
SSLPathPlanner::exePlanForRobot (const int& id)
{
  return true;
}

void
SSLPathPlanner::sendPlanForRobot (const int& id)
{
  if (pub_path_plans[id].getNumSubscribers () > 0)
  {
    unsigned int n_states = solution_data_for_robots[id].size ();
    if (n_states > 0)
    {
      path_plans[id].path_plan.resize (n_states);
      for (unsigned int j = 0; j < n_states; j++)
      {
        const RealVectorStateManifold::StateType *pos = solution_data_for_robots[id][j]->as<
            RealVectorStateManifold::StateType> ();
        geometry_msgs::PoseStamped pose;
        //        std::string frame_id("/");
        //        if (team_ == 0)
        //          frame_id.append ("B");
        //        else
        //          frame_id.append ("Y");
        //        std::stringstream s;
        //        s << (int)id;
        //        frame_id.append (s.str ());
        //        frame_id.append("/");
        //        frame_id.append (ssl::naming::BASE_LINK);
        //        pose.header.frame_id = frame_id;
        pose.header.frame_id = "/field";
        pose.header.stamp = ros::Time::now ();

        pose.pose.position.x = pos->values[0];
        pose.pose.position.y = pos->values[1];
        pose.pose.position.z = 0.0;

        pose.pose.orientation = tf::createQuaternionMsgFromYaw (0);

        path_plans[id].path_plan[j] = pose;
      }
      pub_path_plans[id].publish (path_plans[id]);
    }
  }
}

void
SSLPathPlanner::showPlanForRobot (const int& id)
{
  for (unsigned int i = 0; i < solution_data_for_robots[id].size (); i++)
  {
    const RealVectorStateManifold::StateType *pos = solution_data_for_robots[id][i]->as<
        RealVectorStateManifold::StateType> ();

    std::cout << pos->values[0] << "\t" << pos->values[1] << std::endl;
  }

  unsigned int n_states = solution_data_for_robots[id].size ();
  if (n_states > 0)
  {
    ssl_msgs::RobotPoseStates robot_pose_states;
    robot_pose_states.id = id;
    robot_pose_states.pose_states.resize (n_states);
    for (unsigned int j = 0; j < n_states; j++)
    {
      const RealVectorStateManifold::StateType *pos = solution_data_for_robots[id][j]->as<
          RealVectorStateManifold::StateType> ();
      robot_pose_states.pose_states[j].x = pos->values[0];
      robot_pose_states.pose_states[j].y = pos->values[1];
    }
    team_pose_states.robots.push_back (robot_pose_states);
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
  bounds.setLow (0, -(ssl::config::FIELD_WIDTH_ / 2.0 + 0.5));
  bounds.setHigh (0, (ssl::config::FIELD_WIDTH_ / 2.0 + 0.5));
  bounds.setLow (1, -(ssl::config::FIELD_HEIGHT_ / 2.0 + 0.5));
  bounds.setHigh (1, (ssl::config::FIELD_HEIGHT_ / 2.0 + 0.5));

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

  //do initial plans for the robots according to the control configuration
  for (unsigned int i = 0; i < MAX_N_ROBOTS_PER_TEAM; i++)
  {
    if (robots_control_type[i] != ssl::NO_CONTROL)
    {
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
    for (unsigned int i = 0; i < MAX_N_ROBOTS_PER_TEAM; i++)
    {
      if (robots_control_type[i] != ssl::NO_CONTROL)
        runRobot (i);
    }
    show ();
    ros::spinOnce ();
  }
}

void
SSLPathPlanner::show ()
{
  if (pub_team_path_states.getNumSubscribers () > 0 && team_pose_states.robots.size () > 0)
  {
    pub_team_path_states.publish (team_pose_states);
    team_pose_states.robots.clear ();
  }
}

