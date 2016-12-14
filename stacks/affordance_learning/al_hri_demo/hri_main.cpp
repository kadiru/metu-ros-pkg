#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/Drivers.h>
#include <math.h>
#include <iostream>
#include <sys/time.h>
#include <unistd.h>

#include <actionlib/client/simple_action_client.h>
#include "al_acts/TextToSpeechAction.h"

YARP_DECLARE_DEVICES(icubmod)
;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

typedef actionlib::SimpleActionClient<al_acts::TextToSpeechAction> SpeechClient;

PolyDriver *clientGaze;
IGazeControl *igaze;
SpeechClient* speech_client_;
al_acts::TextToSpeechGoal goal;

#define PI 3.14159265

enum States
{
  LookAtTable, HappyLookHuman, AngryLookHuman, HumanSeated
};

enum SpeechStates
{
  Invalid, Hello, Have_Seat, Over_There
};
float roll_offset, pitch_offset, yaw_offset;
Port emotP;
Bottle outBot;
Vector gazePoint (3);
float gaze_locationX, gaze_locationY;
float roll, pitch, yaw, roll_total = 0, pitch_total = 0, yaw_total = 0;
float x, y, z;
float angleView, absX, absY;
States current_state;
ros::Time t_human_lost;
bool human_lost_first_time = true;
bool have_seated_set = false;
bool human_greeted = false;

const std::string str_hello = "hello";
const std::string str_seat = "please have a seat";
const std::string str_over_there = "Is there any one over there?";
bool did_what_state_requires = false;
SpeechStates curr_speech_state = Hello;

//PROTOTYPES
States
evaluateState (float tanPitch, float tanYaw);
void
doAction (States state);
void
speechAction (SpeechStates state);
SpeechStates
evaluateState2 (float tanPitch, float tanYaw);

int
main (int argc, char** argv)
{

  YARP_REGISTER_DEVICES(icubmod);

  Network::init ();

  Property optGaze ("(device gazecontrollerclient)");
  optGaze.put ("remote", "/iKinGazeCtrl");
  optGaze.put ("local", "/gaze_client");

  clientGaze = new PolyDriver;
  if (!clientGaze->open (optGaze))
  {
    delete clientGaze;
    return false;
  }
  clientGaze->view (igaze);

  ros::init (argc, argv, "gaze_extractor");
  ros::NodeHandle nh;
  tf::TransformListener* tf_listener;
  tf_listener = new tf::TransformListener ();

  speech_client_ = new SpeechClient ("speech_action", true);
  speech_client_->waitForServer ();
  if (speech_client_->isServerConnected ())
    std::cout << "server connected!" << std::endl;

  igaze->setTrackingMode (false);
  igaze->setEyesTrajTime (2.5);
  igaze->setNeckTrajTime (3.0);

  igaze->blockNeckRoll (0.0);

  emotP.open ("/local/emoInt");
  Network::connect ("/local/emoInt", "/icub/face/emotions/in");

  tf::StampedTransform transform_head;
  bool transformation_healthy = false;

  float dToGazeFromHeadX, dToGazeFromHeadY;
  float tanPitch, tanYaw;

  btScalar r, p, ya;

  outBot.clear ();
  outBot.addString ("set");
  outBot.addString ("all");
  outBot.addString ("hap");
  emotP.write (outBot);

  int input;

  /*
   * CALIBRATION PROCESS START
   */
  cout << "Say experimenter to look directly forward" << endl;
  cout << "Press enter a key to start calibration process:" << endl;
  cin >> input;
  for (int count = 0; count < 20; count++)
  {
    try
    {
      tf_listener->lookupTransform ("base_footprint", "hat_link", ros::Time (0), transform_head);
      transformation_healthy = true;
    }
    catch (tf::TransformException ex)
    {
      transformation_healthy = false;
    }

    transform_head.getBasis ().getRPY (r, p, ya);

    roll_total += r;
    pitch_total += p;
    yaw_total += ya;

  }

  roll_offset = roll_total / 20;
  pitch_offset = pitch_total / 20;
  yaw_offset = yaw_total / 20;
  /*
   * CALIBRATION PROCESS END
   */

  States received_state;
  current_state = States (2);
  bool first_time = true;
  struct timeval start, end;
  long mtime, seconds, useconds;

  ros::Rate rate (50);
  while (nh.ok ())
  {

    try
    {
      tf_listener->lookupTransform ("base_footprint", "hat_link", ros::Time (0), transform_head);
      transformation_healthy = true;
    }
    catch (tf::TransformException ex)
    {
      transformation_healthy = false;
    }

    if (transformation_healthy)
    {
      transform_head.getBasis ().getRPY (r, p, ya);
      roll = r;
      pitch = p;
      yaw = ya;

      x = transform_head.getOrigin ().x ();
      y = transform_head.getOrigin ().y ();
      z = transform_head.getOrigin ().z ();

      std::cout << "x " << x << "\ty" << y << "\tz" << z << "\troll" << roll << "\tpitch" << pitch << "\tyaw" << yaw
          << std::endl;

      tanPitch = tan (pitch - pitch_offset);
      if (tanPitch != 0)
        dToGazeFromHeadX = (z + 0.2) / tanPitch;

      tanYaw = tan (yaw - yaw_offset);
      dToGazeFromHeadY = tanYaw * dToGazeFromHeadX;

      gaze_locationX = x + dToGazeFromHeadX - 0.6;
      gaze_locationY = y + dToGazeFromHeadY;
      //if(gaze_locationY < 0)
      //gaze_locationY += 0.1;
      //else
      //gaze_locationY -= 0.1;
      speechAction (evaluateState2 (tanPitch, tanYaw));
      received_state = evaluateState (tanPitch, tanYaw);
      if (current_state != received_state)
      {
        if (first_time)
        {
          gettimeofday (&start, NULL);
          first_time = false;
        }
        else
        {
          gettimeofday (&end, NULL);
          seconds = end.tv_sec - start.tv_sec;
          useconds = end.tv_usec - start.tv_usec;

          mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

          if (mtime > 750)
          {
            current_state = received_state;
            first_time = true;
            doAction (received_state);
          }

        }
      }
      else
      {
        doAction (current_state);
        first_time = true;
      }
    }
    else
    {

      if (!human_lost_first_time)
      {
        human_lost_first_time = true;
        t_human_lost = ros::Time::now ();
      }
      else
      {
        if ((ros::Time::now () - t_human_lost) > ros::Duration (5))
        {
          //is there anyone out there
          t_human_lost = ros::Time::now ();
        }
      }
    }

    ros::spinOnce ();
    rate.sleep ();
  }

  return 0;
}

States
evaluateState (float tanPitch, float tanYaw)
{
  //Table Constraints x: From -21 to -65 and y: From -35 to 35 z: -20
  if (gaze_locationX < -0.0 && gaze_locationX > -1.0 && gaze_locationY < 0.5 && gaze_locationY > -0.5)
  {
    return LookAtTable;
  }
  else
  {
    absY = fabs (x);
    absX = fabs (y);
    angleView = atan2 (absY, absX);

    if (y < 0 && yaw - yaw_offset > PI / 2 - angleView - PI / 9 && yaw - yaw_offset < PI / 2 - angleView + PI / 9)
    {
      absY = fabs (z - 0.35);
      absX = fabs (x);
      angleView = atan2 (absY, absX);
      if (z > 0.35 && pitch - pitch_offset > angleView - PI / 7 && pitch - pitch_offset < angleView - PI / 18)
      {
        return HappyLookHuman;
      }
      else if (z < 0.35 && pitch - pitch_offset > -angleView - PI / 7 && pitch - pitch_offset < -angleView + PI / 7)
      {
        return HappyLookHuman;
      }
      else
      {
        return AngryLookHuman;
      }
    }
    else if (y > 0 && yaw - yaw_offset < -PI / 2 + angleView + PI / 9 && yaw - yaw_offset > -PI / 2 + angleView - PI
        / 9)
    {
      absY = fabs (z - 0.35);
      absX = fabs (x);
      angleView = atan2 (absY, absX);
      if (z > 0.35 && pitch - pitch_offset > angleView - PI / 7 && pitch - pitch_offset < angleView - PI / 18)
      {
        return HappyLookHuman;
      }
      else if (z < 0.35 && pitch - pitch_offset > -angleView - PI / 7 && pitch - pitch_offset < -angleView + PI / 7)
      {
        return HappyLookHuman;
      }
      else
      {
        return AngryLookHuman;
      }

    }
    else
    {
      return AngryLookHuman;
    }

  }
}

SpeechStates
evaluateState2 (float tanPitch, float tanYaw)
{
  //Table Constraints x: From -21 to -65 and y: From -35 to 35 z: -20
  absY = fabs (x);
  absX = fabs (y);
  angleView = atan2 (absY, absX);

  //Robot at yaw
  if (sqrt (x * x + y * y) > 1.3)
  {
    std::cout << curr_speech_state << " " << Over_There << std::endl;
    return Over_There;
  }
  if (y < 0 && yaw - yaw_offset > PI / 2 - angleView - PI / 9 && yaw - yaw_offset < PI / 2 - angleView + PI / 9)
  {
    absY = fabs (z - 0.35);
    absX = fabs (x);
    angleView = atan2 (absY, absX);

    if (z > 0.5 && pitch - pitch_offset > angleView - PI / 7 && pitch - pitch_offset < angleView - PI / 18)
    {
      std::cout << curr_speech_state << " " << Have_Seat << std::endl;
      return Have_Seat;
    }
    else if (z < 0.5 && pitch - pitch_offset > -angleView - PI / 7 && pitch - pitch_offset < -angleView + PI / 7)
    {
      std::cout << curr_speech_state << " " << Invalid << std::endl;
      return Invalid;
    }
    else
    {
      std::cout << curr_speech_state << " " << Hello << std::endl;
      return Hello;
    }
  }
  else if (y > 0 && yaw - yaw_offset < -PI / 2 + angleView + PI / 9 && yaw - yaw_offset > -PI / 2 + angleView - PI / 9)
  {
    absY = fabs (z - 0.35);
    absX = fabs (x);
    angleView = atan2 (absY, absX);
    if (z > 0.5 && pitch - pitch_offset > angleView - PI / 7 && pitch - pitch_offset < angleView - PI / 18)
    {
      std::cout << curr_speech_state << " " << Have_Seat << std::endl;
      return Have_Seat;
    }
    else if (z < 0.5 && pitch - pitch_offset > -angleView - PI / 7 && pitch - pitch_offset < -angleView + PI / 7)
    {
      std::cout << curr_speech_state << " " << Invalid << std::endl;
      return Invalid;
    }
    else
    {
      std::cout << curr_speech_state << " " << Hello << std::endl;
      return Hello;
    }

  }
  else
  {
    std::cout << curr_speech_state << " " << Hello << std::endl;
    return Hello;
  }

}

void
doAction (States state)
{
  if (state == LookAtTable)
  {
    //    have_seated_set = false;
    //    human_greeted = false;
    outBot.clear ();
    outBot.addString ("set");
    outBot.addString ("all");
    outBot.addString ("hap");
    emotP.write (outBot);

    if (gaze_locationX < -0.65)
      gazePoint[0] = -0.65;
    else if (gaze_locationX > -0.21)
      gazePoint[0] = -0.24;
    else
      gazePoint[0] = gaze_locationX;

    if (gaze_locationY > 0.35)
      gazePoint[1] = 0.30;
    else if (gaze_locationY < -0.35)
      gazePoint[1] = -0.30;
    else
      gazePoint[1] = gaze_locationY;

    gazePoint[2] = -0.2;

    igaze->lookAtFixationPoint (gazePoint);
  }
  else if (state == HappyLookHuman)
  {

    //    if (!have_seated_set)
    //    {
    //      goal.text = str_seat;
    //      speech_client_->sendGoal (goal);
    //
    //      have_seated_set = true;
    //      human_greeted = false;
    //    }
    gazePoint[0] = x;
    gazePoint[1] = y;
    gazePoint[2] = z - 0.25;

    outBot.clear ();
    outBot.addString ("set");
    outBot.addString ("all");
    outBot.addString ("hap");
    emotP.write (outBot);

    igaze->lookAtFixationPoint (gazePoint);
  }
  else
  {
    //    if (!human_greeted)
    //    {
    //      goal.text = str_hello;
    //      speech_client_->sendGoal (goal);
    //
    //      //Say hello a seat
    //      human_greeted = true;
    //      have_seated_set = false;
    //    }
    gazePoint[0] = x;
    gazePoint[1] = y;
    gazePoint[2] = z - 0.25;

    outBot.clear ();
    outBot.addString ("set");
    outBot.addString ("all");
    outBot.addString ("evi");
    emotP.write (outBot);

    igaze->lookAtFixationPoint (gazePoint);
  }
}

void
speechAction (SpeechStates state)
{
  if (state == Invalid)
  {
    curr_speech_state = state;
  }
  else if (state == Over_There)
  {
    if (state != curr_speech_state)
    {
      curr_speech_state = state;
      did_what_state_requires = false;
    }
    if (!did_what_state_requires)
    {
      did_what_state_requires = true;
      goal.text = str_over_there;
      speech_client_->sendGoal (goal);
    }
  }
  else if (state == Hello)
  {
    if (state != curr_speech_state)
    {
      curr_speech_state = state;
      did_what_state_requires = false;
    }
    if (!did_what_state_requires)
    {
      did_what_state_requires = true;
      goal.text = str_hello;
      speech_client_->sendGoal (goal);
    }
  }
  else
  {
    if (state != curr_speech_state)
    {
      curr_speech_state = state;
      did_what_state_requires = false;
    }
    if (!did_what_state_requires)
    {
      did_what_state_requires = true;
      goal.text = str_seat;
      speech_client_->sendGoal (goal);
    }
  }
}
