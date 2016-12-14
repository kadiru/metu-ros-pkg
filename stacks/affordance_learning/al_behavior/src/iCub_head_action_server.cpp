#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <al_behavior/HeadAction.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <time.h>
#include <stdio.h>

#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <gsl/gsl_math.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/all.h>
#include <yarp/math/api.h>

#include <iCub/ctrl/math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>

#define CTRL_THREAD_PER     0.02        // [s]
#define PRINT_STATUS_PER    1.0         // [s]

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class HeadAction: public RateThread,
                  public GazeEvent {
public:
	HeadAction(std::string name) :
	as_(nh_, name, boost::bind(&HeadAction::processCB, this, _1), false), action_name_(name), RateThread(int(CTRL_THREAD_PER * 1000)) {
		//register the goal and feedback callbacks
        as_.start();
	}

	~HeadAction() {
	}

	virtual bool threadInit() {
		Property optGaze("(device gazecontrollerclient)");
		optGaze.put("remote","/iKinGazeCtrl");
		optGaze.put("local","/gaze_client");

		if (!clientGaze.open(optGaze)) {
			return false;
		}

		// open the view
		clientGaze.view(igaze);

		igaze->storeContext(&startup_context_id);

		// set trajectory time:
		igaze->setNeckTrajTime(5);
		igaze->setEyesTrajTime(5);

		igaze->setTrackingMode(true);

		fp.resize(3);

		return true;
	}

	virtual void afterStart(bool s) {
		if (s)
			fprintf(stdout,"Thread started successfully\n");
		else
			fprintf(stdout,"Thread did not start\n");
	}

	void run() {
		ros::Rate r(1);

		while(true) {
			std::cout << "Running" << std::endl;
			ros::spinOnce();
			r.sleep();
		}
	}

	void processCB(const al_behavior::HeadGoalConstPtr& goal) {
		ros::Rate r(1);
		bool success = true;
		bool ok = false;
		Vector curr_angle(3);

		fp[0] = goal->goalPositon.x;
		fp[1] = goal->goalPositon.y;
		fp[2] = goal->goalPositon.z;

		igaze->lookAtFixationPoint(fp);
		igaze->checkMotionDone(&ok);

		while(!ok) {
			if (as_.isPreemptRequested()) {
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				success = false;
				break;
			}

			cout << "Server is running.." << endl;

			igaze->getAngles(curr_angle);

			feedback_.feedback.x = curr_angle[0];
			feedback_.feedback.y = curr_angle[1];
			feedback_.feedback.z = curr_angle[2];

			as_.publishFeedback(feedback_);

			igaze->checkMotionDone(&ok);

			r.sleep();
		}

		if(success) {
			result_.lastPosition = feedback_.feedback;
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
	}

protected:
	virtual void gazeEventCallback() {
		Vector ang;
		igaze->getAngles(ang);

		fprintf(stdout,"Actual gaze configuration: (%s) [deg]\n", ang.toString(3,3).c_str());
	}

	virtual void threadRelease() {
		// we require an immediate stop
		// before closing the client for safety reason
		igaze->stopControl();

		// it's a good rule to restore the controller
		// context as it was before opening the module
		igaze->restoreContext(startup_context_id);

		cout << "Closing..." << endl;

		clientGaze.close();
	}

private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<al_behavior::HeadAction> as_;
	std::string action_name_;
	al_behavior::HeadFeedback feedback_;
	al_behavior::HeadResult result_;

	PolyDriver        clientGaze;
	IGazeControl     *igaze;
	IEncoders        *ienc;
	IPositionControl *ipos;

	int state;
	int startup_context_id;

	Vector fp;
};

class CtrlModule: public RFModule
{
protected:
    HeadAction *thr;
    std::string name;

public:
    CtrlModule(std::string name):
    name(name) {
    }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new HeadAction(name);
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
	ros::init(argc, argv, "iCub_head_action_server");

	YARP_REGISTER_DEVICES(icubmod)

	Network yarp;

	if (!yarp.checkNetwork()) {
		fprintf(stdout,"Error: yarp server does not seem available\n");
		return -1;
	}

	CtrlModule mod("head_action");

	ResourceFinder rf;
	return mod.runModule(rf);

	return 0;
}
