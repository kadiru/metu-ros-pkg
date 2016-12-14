/*  Copyright (C) 2011, Kadir Firat Uyanik 
 *	Middle East Technical University, Kovan Research Lab
 *	kadir@ceng.metu.edu.tr
 *
 *	http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  main.cpp is part of ssl_game_planner.
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

#include "ssl_game_planner/SSLGamePlanner.h"

int
main (int argc, char** argv)
{
  //team of the game_planner should be provided
  if (argc < 2)
  {
    std::cerr << "usage: " << std::endl;
    std::cerr << "./ssl_game_planner <team>" << std::endl;
    std::cerr << "<team>: <0> for blue, <1> for yellow" << std::endl;
    exit (-1);
  }
  char* team = argv[1];
  std::string team_name;

  if (atoi (team) == 0)
    team_name.assign (ssl::naming::entity::BLUE_TEAM);
  else
    team_name.assign (ssl::naming::entity::YELLOW_TEAM);

  ros::init (argc, argv, team_name + "_ssl_game_planner");
  ros::NodeHandle n;
  n.setParam ("team", team_name);
  n.setParam ("sub_global_state_topic_name", ssl::naming::topic::ESTIMATED_GLOBAL_ST);

  SSLGamePlanner* ssl_game_planner = new SSLGamePlanner (n);
  ssl_game_planner->run ();

  return 0;
}
