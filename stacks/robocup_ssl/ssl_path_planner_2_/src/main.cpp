/*  Copyright (C) 2011, Kadir Firat Uyanik 
 *	Middle East Technical University, Kovan Research Lab
 *	kadir@ceng.metu.edu.tr
 *
 *	http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  main.cpp is part of ssl_path_planner.
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

#include "ssl_path_planner/SSLPathPlanner.h"

int main(int argc, char** argv)
{
  //team of the planner should be provided
  if(argc<2)
  {
    std::cerr<<"usage: "<<std::endl;
    std::cerr<<"./dummy_game_planner <team>"<<std::endl;
    std::cerr<<"<team>: 0 for blue, 1 for yellow"<<std::endl;
    exit(-1);
  }
  char* team = argv[1];
  std::string name;
  if(atoi(team) == 0)
    name.assign(BLUE_TEAM_NAME);
  else
    name.assign(YELLOW_TEAM_NAME);
  name.append("_path_planner");
  std::cout<<name<<" node started"<<std::endl;

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  SSLPathPlanner* ssl_path_planner=new SSLPathPlanner(nh,atoi(team));
  ssl_path_planner->run();
  return 0;
}
