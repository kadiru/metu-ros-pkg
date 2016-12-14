#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <al_behavior/ArmAction.h>
#include <al_msgs/JointCmd.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IEncoders.h>

#include <gsl/gsl_math.h>

#include <stdio.h>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class ArmAction: public RateThread,
				 public CartesianEvent {
public:
	ArmAction(std::string name, bool left_hand):
		as_(nh_, name, boost::bind(&ArmAction::processCB, this, _1), false), action_name_(name),
		RateThread(int(CTRL_THREAD_PER * 1000)), left_hand_(left_hand) {
		as_.start();
	}

	virtual bool threadInit() {
		Property option, option_arm;

		if(left_hand_) {
			option.put("device", "cartesiancontrollerclient");
			option.put("remote","/icubSim/cartesianController/left_arm");
			option.put("local","/cartesian_client/left_arm");

			option_arm.put("device", "remote_controlboard");
			option_arm.put("remote", "/icubSim/left_arm");
			option_arm.put("local", "/arm_client/left_arm");
		}
		else {
			option.put("device", "cartesiancontrollerclient");
			option.put("remote","/icubSim/cartesianController/right_arm");
			option.put("local","/cartesian_client/right_arm");

			option_arm.put("device", "remote_controlboard");
			option_arm.put("remote", "/icubSim/right_arm");
			option_arm.put("local", "/arm_client/right_arm");
		}

		if(!arm.open(option_arm))
			return false;

		bool ok;
		ok = arm.view(iarm);
		ok &= arm.view(ivel);
		ok &= arm.view(ienc);

		if (!ok) {
			printf("Problems acquiring interfaces\n");
			return false;
		}

		iarm->getAxes(&n_joints);

		//TODO: Control the velocity and acceleration values of joints
		for(uint i = 0; i < n_joints; i++) {
			iarm->setRefSpeed(i, 20.0);
		}

		if (!cart.open(option))
			return false;

		// open the view
		cart.view(icart);

		// latch the controller context in order to preserve
		// it after closing the module
		// the context contains the dofs status, the tracking mode,
		// the resting positions, the limits and so on.
		icart->storeContext(&startup_context_id);

		// set trajectory time
		icart->setTrajTime(3.0);

		// get the torso dofs
		Vector newDof, curDof;
		icart->getDOF(curDof);
		newDof=curDof;

		// enable the torso yaw and pitch
		// disable the torso roll
		newDof[0]=1;
		newDof[1]=1;
		newDof[2]=1;

		// impose some restriction on the torso pitch
		limitTorsoPitch();

		// send the request for dofs reconfiguration
		icart->setDOF(newDof,curDof);

		// print out some info about the controller
		Bottle info;
		icart->getInfo(info);
		fprintf(stdout,"info = %s\n",info.toString().c_str());

		// register the event, attaching the callback
		icart->registerEvent(*this);

		xd.resize(3);
		od.resize(4);

		return true;
	}

	void limitTorsoPitch() {
		int axis = 0; // pitch joint
		double min, max;

		// we keep the lower limit
		icart->getLimits(axis,&min,&max);
		icart->setLimits(axis,min,MAX_TORSO_PITCH);
	}

	void run() {
		ros::Rate r(1);

		while(nh_.ok()) {
			ROS_INFO("Running...");
			ros::spinOnce();
			r.sleep();
		}
	}

	void processCB(const al_behavior::ArmGoalConstPtr& goal) {
		ros::Rate r(5);
		bool success = true;
		bool ok = false;
		double* curr_angle = new double[n_joints];
		Vector curr_xd(3);
		Vector curr_od(4);
		al_msgs::JointCmd cmd;

		feedback_.feed_joint_pos.clear();
		result_.result_joint_pos.clear();

		if(goal->goal_mode == true) { //Cartesian Control
			ROS_INFO("Cartesian Control Mode is selected..");

			this->xd[0] = goal->goal_cartesian_pos.position.x;
			this->xd[1] = goal->goal_cartesian_pos.position.y;
			this->xd[2] = goal->goal_cartesian_pos.position.z;

			this->od[0] = goal->goal_cartesian_pos.orientation.x;
			this->od[1] = goal->goal_cartesian_pos.orientation.y;
			this->od[2] = goal->goal_cartesian_pos.orientation.z;
			this->od[3] = goal->goal_cartesian_pos.orientation.w;

			ROS_INFO("xd: %s", xd.toString().c_str());
			ROS_INFO("od: %s", od.toString().c_str());

			icart->goToPoseSync(this->xd, this->od);
			icart->checkMotionDone(&ok);
		}
		else { //Joint Control
			ROS_INFO("Joint Control Mode is selected..");

			ROS_INFO("The JointCmd size: %d", goal->goal_joint_pos.size());
			for(uint i = 0; i < goal->goal_joint_pos.size(); i++) {
				ROS_INFO("joint_id: %d, joint_val: %lf", goal->goal_joint_pos[i].joint_id, goal->goal_joint_pos[i].joint_val);
				iarm->positionMove(goal->goal_joint_pos[i].joint_id, goal->goal_joint_pos[i].joint_val);
			}

			iarm->checkMotionDone(&ok);
		}

		while(!ok) {
			if(as_.isPreemptRequested()) {
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				success = false;

				if(goal->goal_mode) {
					icart->stopControl();
				}
				else {
					ienc->getEncoders(curr_angle);
					iarm->positionMove(curr_angle);
				}

				break;
			}

			ROS_INFO("PROCESS IS ACTIVE..");

			ok = false;
			ROS_INFO("ok variable is false again..");

			if(goal->goal_mode == true) { //Cartesian Control
				icart->getPose(curr_xd, curr_od);

				ROS_INFO("CartesianControl feedback_.feed_cartesian_pos.position is being recorded");
				feedback_.feed_mode = goal->goal_mode;
				feedback_.feed_cartesian_pos.position.x = curr_xd[0];
				feedback_.feed_cartesian_pos.position.y = curr_xd[1];
				feedback_.feed_cartesian_pos.position.z = curr_xd[2];
				ROS_INFO("CartesianControl feedback_.feed_cartesian_pos.position is recorded");

				ROS_INFO("CartesianControl feedback_.feed_cartesian_pos.orientation is being recorded");
				feedback_.feed_cartesian_pos.orientation.x = curr_od[0];
				feedback_.feed_cartesian_pos.orientation.y= curr_od[1];
				feedback_.feed_cartesian_pos.orientation.z = curr_od[2];
				feedback_.feed_cartesian_pos.orientation.w = curr_od[3];
				ROS_INFO("CartesianControl feedback_.feed_cartesian_pos.orientation is recorded");

				ROS_INFO("CartesianControl checkMotionDone().begin");
				icart->checkMotionDone(&ok);
				ROS_INFO("CartesianControl checkMotionDone().end");
			}
			else { //Joint Control
				feedback_.feed_joint_pos.clear();

				ienc->getEncoders(curr_angle);

				for(uint i = 0; i < n_joints; i++) {
					ROS_INFO("JointControl feed_joint_pos is being recorded %d", i);
					cmd.joint_id = i;
					cmd.joint_val = curr_angle[i];

					feedback_.feed_joint_pos.push_back(cmd);
					ROS_INFO("JointControl feed_joint_pos is recorded %d", i);
				}

				ROS_INFO("JointControl checkMotionDone().begin");
				iarm->checkMotionDone(&ok);
				ROS_INFO("JointControl checkMotionDone().end");
			}

			as_.publishFeedback(feedback_);

			r.sleep();
		}

		result_.result_mode = feedback_.feed_mode;

		if(result_.result_mode == true) {
			result_.result_cartesian_pos.position.x = feedback_.feed_cartesian_pos.position.x;
			result_.result_cartesian_pos.position.y = feedback_.feed_cartesian_pos.position.y;
			result_.result_cartesian_pos.position.z = feedback_.feed_cartesian_pos.position.z;

			result_.result_cartesian_pos.orientation.x = feedback_.feed_cartesian_pos.orientation.x;
			result_.result_cartesian_pos.orientation.y = feedback_.feed_cartesian_pos.orientation.y;
			result_.result_cartesian_pos.orientation.z = feedback_.feed_cartesian_pos.orientation.z;
			result_.result_cartesian_pos.orientation.w = feedback_.feed_cartesian_pos.orientation.w;
		}
		else {
			result_.result_cartesian_pos = feedback_.feed_cartesian_pos;

			for(uint i = 0; i < feedback_.feed_joint_pos.size(); i++) {
				cmd.joint_id = feedback_.feed_joint_pos[i].joint_id;
				cmd.joint_val = feedback_.feed_joint_pos[i].joint_val;

				result_.result_joint_pos.push_back(cmd);
			}
		}

		if(success) {
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
		else {
			ROS_INFO("%s: Aborted", action_name_.c_str());
			//set the action state to aborted
			as_.setAborted(result_);
		}

		ROS_INFO("Exiting from ArmAction::processCB()");
	}

	virtual void cartesianEventCallback() {
		fprintf(stdout,"20%% of trajectory attained\n");
	}

	virtual void afterStart(bool s) {
		if (s)
			fprintf(stdout,"Thread started successfully\n");
		else
			fprintf(stdout,"Thread did not start\n");
	}

private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<al_behavior::ArmAction> as_;
	std::string action_name_;
	bool left_hand_;
	al_behavior::ArmFeedback feedback_;
	al_behavior::ArmResult result_;

	PolyDriver         cart, arm;

	ICartesianControl *icart;

	IPositionControl* iarm;
	IVelocityControl* ivel;
	IEncoders* ienc;
	int n_joints;

	Vector xd;
	Vector od;

	int startup_context_id;
};

class CtrlModule: public RFModule
{
protected:
    ArmAction *thr;
    std::string name_;
    bool left_arm_;
    std::string part;

public:
    CtrlModule(std::string name, bool left_arm):
    name_(name), left_arm_(left_arm) {
    }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();


        thr=new ArmAction(name_, left_arm_);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};

int main(int argc, char** argv) {

	std::string part = "";
	bool l_arm;
	part.assign(argv[1]);
	ros::init(argc, argv, "iCub_" + part + "_arm_action_server");

	if(part == "left") {
		l_arm = true;
	}
	else if(part == "right") {
		l_arm = false;
	}
	else {
		std::cout << "Invalid part name.." << std::endl;
		return -1;
	}

	YARP_REGISTER_DEVICES(icubmod)

	Network yarp;

	if (!yarp.checkNetwork()) {
		fprintf(stdout,"Error: yarp server does not seem available\n");
		return -1;
	}

	CtrlModule mod(part + "_arm_action", l_arm);

	ResourceFinder rf;
	return mod.runModule(rf);

	return 0;
}
