#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <al_behavior/ArmAction.h>
#include <al_msgs/JointCmd.h>

using namespace std;

bool test_arms(al_behavior::ArmGoal& goal) {
	int mode;
	double x, y, z;
	double o_x, o_y, o_z, o_w = M_PI;
	int j_id;
	double j_val;
	al_msgs::JointCmd cmd;

	std::cout << "Set mode (1:Cartesian, 0:Joint, 2:Exit):" << std::endl;
	cin >> mode;

	if(mode == 1) {
		goal.goal_mode = true;

		cout << "Enter position where iCub arm move: (x y z)" << endl;
		cin >> x >> y >> z;
		goal.goal_cartesian_pos.position.x = x;
		goal.goal_cartesian_pos.position.y = y;
		goal.goal_cartesian_pos.position.z = z;

		cout << "Enter orientation where iCub arm move: (o_x o_y o_z)" << endl;
		cin >> o_x >> o_y >> o_z;
		goal.goal_cartesian_pos.orientation.x = o_x;
		goal.goal_cartesian_pos.orientation.y = o_y;
		goal.goal_cartesian_pos.orientation.z = o_z;
		goal.goal_cartesian_pos.orientation.w = o_w;

	}
	else if(mode == 0) {
		goal.goal_mode = false;
		goal.goal_joint_pos.clear();

		while(true) {
			cout << "Enter joint id and joint value (joint_id joint_val) or -1 to finish" << endl;
			cin >> j_id;

			if(j_id != -1) {
				cin >> j_val;
				cmd.joint_id = j_id;
				cmd.joint_val = j_val;

				goal.goal_joint_pos.push_back(cmd);
			}
			else
				break;
		}

	}
	else if(mode == 2) {
		return false;
	}

	return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "iCub_left_arm_action_client");
  al_behavior::ArmGoal goal;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<al_behavior::ArmAction> ac("left_arm_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action

  while(true) {
//	  std::cout << "Enter position where iCub looks: (x y z)" << std::endl;
//	  std::cout << "Enter -1 in order to finalize" << std::endl;
//	  std::cin >> x;
//
//	  if(x == -1)
//		  break;
//
//	  std::cin >> y >> z;
//	  goal.goalPositon.x = x;
//	  goal.goalPositon.y = y;
//	  goal.goalPositon.z = z;
//
//	  ac.sendGoal(goal);


	  if(test_arms(goal)) {
		  ac.sendGoal(goal);
	  }
	  else {
		  break;
	  }

	  bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

	  if(finished_before_timeout) {
		  actionlib::SimpleClientGoalState state = ac.getState();
		  ROS_INFO("Action finished: %s",state.toString().c_str());
	  }
	  else {
		  ac.cancelGoal();
		  ROS_INFO("Action did not finish before the time out.");
	  }
  }

  //exit
  return 0;
}
