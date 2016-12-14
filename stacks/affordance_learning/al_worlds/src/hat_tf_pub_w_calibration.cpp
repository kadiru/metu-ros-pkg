/*
 * hat_tf_pub.cpp
 * Copyright (c) 2012, Kadir Firat Uyanik, KOVAN Research Lab, METU
 * kadir@ceng.metu.edu.tr
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of KOVAN Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//ros includes
#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/JointState.h>

// yarp includes
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <pthread.h>

// standard includes
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include "visualization_msgs/Marker.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

const string CHANNEL201 = "Channel201";
const string CHANNEL202 = "Channel202";
const string CHANNEL203 = "Channel203";
const string CHANNEL204 = "Channel204";

typedef struct ledVector
{
  ConstString channelName;
  Vector coordinates;

  ledVector ()
  {
    coordinates.resize (3);
  }
} LEDVector;

typedef struct transform
{
  Vector rpy;
  Vector offset;
  transform ()
  {
    rpy.resize (3);
    offset.resize (3);
  }
} Transform;

vector<LEDVector> vectorList;
Vector rpy (3);
Vector offset (3);
Transform avg_tranform;
Transform prev_tranform;
tf::StampedTransform* avg_tf = NULL;
bool tf_calculated = false;
bool calibrated = false;
bool calibration_started = false;
bool first_tf_calculated = false;
visualization_msgs::Marker head_vector;
int calibration_counter = 0;
btScalar r, p, ya;
float roll_total = 0,pitch_total = 0, yaw_total = 0;
float roll_offset,pitch_offset, yaw_offset;

const int N_TRANSFORM = 3;

bool
getCoordinatesByChannelName (const char* channelOfInterest, Vector & vector);

bool
getCameraLedInfo (const Bottle& b);

bool
getTransformation (tf::Transform &transform);

//tf::Transform
//getTF (const Vector& offset, const Vector& rpy);

//bool
//calcTF (Vector& offset, Vector& rpy, tf::Transform& transform);

//mocap data received callback
class DataPort : public BufferedPort<Bottle>
{
  virtual void
  onRead (Bottle& b)
  {
    // process data in b
    bool safe_tf = getCameraLedInfo (b);

    if (safe_tf)
    {
      tf::Transform transform;
	 
      if (getTransformation (transform))
      {
//        std::cout << "translation: " << transform.getOrigin ().x () << " " << transform.getOrigin ().y () << " "
//            << transform.getOrigin ().z () << std::endl;
//        double r, p, y;
//        transform.getBasis ().getRPY (r, p, y);
//        std::cout << "orientation: " << r << " " << p << " " << y << std::endl;

        if (!first_tf_calculated)
        {
          first_tf_calculated = true;
          avg_tf = new tf::StampedTransform (transform, ros::Time::now (), "/base_footprint", "/hat_link");
          
        }
        else if(!calibrated && calibration_started)
        {
		  
		  transform.getBasis ().getRPY (r, p, ya);
		  
          //avg_tf->setOrigin (transform.getOrigin ());
          //avg_tf->setRotation (transform.getRotation ());
          //avg_tf->stamp_ = ros::Time::now ();
          
          roll_total += r;
          pitch_total += p;
          yaw_total += ya;
          
          calibration_counter++;
          if(calibration_counter == 20)
          {
			  roll_offset = roll_total/calibration_counter;
			  pitch_offset = pitch_total/calibration_counter;
			  yaw_offset = yaw_total/calibration_counter;
			  calibrated = true;
		  }
			
        }
        else if(calibrated)
        {
			avg_tf->setOrigin (transform.getOrigin ());
			avg_tf->setRotation (transform.getRotation ());
			avg_tf->stamp_ = ros::Time::now ();
		}
      }
    }
  }
};

int
main (int argc, char* argv[])
{
  //ros initializations
  ros::init (argc, argv, "mocap_hat");
  ros::NodeHandle n;
  ros::Publisher pub_marker;
  ros::Publisher pub_js;
  pub_marker = n.advertise<visualization_msgs::Marker> ("hat_marker", 10);
  //  pub_js = n.advertise<sensor_msgs::JointState> ("human_joint_states", 10);

  //yarp initializations
  Network::init ();
  DataPort vzInPort;
  vzInPort.useCallback ();
  vzInPort.open ("/i:vzListen");
  Network::connect ("/vzRawPout", "/i:vzListen");

  std::cout << "waiting for mocap to be connected" << std::endl;
  while (!first_tf_calculated && n.ok ())
  {
  }
  std::cout << "tf is calculated" << std::endl;

  //  head_vector.color.a = 1.0;
  //  head_vector.color.r = 0.0;
  //  head_vector.color.g = 1.0;
  //  head_vector.color.b = 1.0;
  //  head_vector.id = 1;
  //  //  head_vector.lifetime
  //  head_vector.ns = "hat";
  //  head_vector.scale.x = 0.20;
  //  head_vector.scale.y = 0.20;
  //  head_vector.scale.z = 0.20;
  //  head_vector.type = visualization_msgs::Marker::LINE_STRIP;
  //  head_vector.action = visualization_msgs::Marker::ADD;
  //  head_vector.header.frame_id = "base_footprint";
  //  head_vector.header.stamp = ros::Time::now ();
  int input;
  std::cout<<"Say experimenter to look directly forward"<<std::endl;
  std::cout<<"Press enter a key to start calibration process:"<<std::endl;
  std::cin>>input;
  while(n.ok () && !calibrated)
  {
	  calibration_started = true;
	  ros::spinOnce ();
  }
  std::cout<<"Calibration Finished"<<std::endl;
  
  static tf::TransformBroadcaster br;
  //  sensor_msgs::JointState js;

  ros::Rate r (50);
  while (n.ok () && calibrated)
  {
    if (avg_tf != NULL)
    {
      br.sendTransform (tf::StampedTransform (*avg_tf, ros::Time::now (), "/base_footprint", "/hat_link"));
      //      js.header.stamp = ros::Time::now ();
      //      js.name.resize (3);
      //      js.position.resize (3);
      //      js.name[0] = "head_1_to_head_roll";
      //      js.name[1] = "head_roll_to_head_pitch";
      //      js.name[2] = "head_pitch_to_head_yaw";
      //
      //      btMatrix3x3 hat_frame_rot;
      //      tf::Quaternion q = avg_tf->getRotation ();
      //      hat_frame_rot.setRotation (q);
      //      double roll, pitch, yaw;
      //      hat_frame_rot.getRPY (roll, pitch, yaw);
      //      js.position[0] = roll;
      //      js.position[1] = pitch;
      //      js.position[2] = yaw;
      //
      //      if (pub_js.getNumSubscribers ())
      //        pub_js.publish (js);

      //      head_vector.pose.position.x = avg_tf->getOrigin ().x ();
      //      head_vector.pose.position.y = avg_tf->getOrigin ().y ();
      //      head_vector.pose.position.z = avg_tf->getOrigin ().z ();
      //
      //      head_vector.pose.orientation.w = avg_tf->getRotation ().w ();
      //      head_vector.pose.orientation.x = avg_tf->getRotation ().x ();
      //      head_vector.pose.orientation.y = avg_tf->getRotation ().y ();
      //      head_vector.pose.orientation.z = avg_tf->getRotation ().z ();

      //      pub_marker.publish (head_vector);
    }

    ros::spinOnce ();
    r.sleep ();
  }
  Network::fini ();
  delete avg_tf;
  return 0;
}

bool
getCoordinatesByChannelName (const char* channelOfInterest, Vector & vector)
{
  for (uint i = 0; i < vectorList.size (); i++)
    if (vectorList[i].channelName == channelOfInterest)
    {
      vector = vectorList[i].coordinates;
      std::string s (channelOfInterest);
//      std::cout << s << std::endl;
      return true;
    }
  std::string s (channelOfInterest);
  //  cerr << "Could not find the led " << s << "something wrong with this led or mocap, EXITING!" << endl;
  return false;
  //  exit (-1);
}

bool
getCameraLedInfo (const Bottle& b)
{
  int channelCount = b.get (0).asInt ();//get number of active leds
//  std::cout << "n leds: " << channelCount << std::endl;

  vectorList.clear ();
  for (int i = 1; i < channelCount * 5; i = i + 5)
  {
    LEDVector led;
    led.channelName = b.get (i).asString ();//channel name
    //convert vz to icub frame
    led.coordinates[0] = -b.get (i + 1).asDouble ();// VZ::-x -> icub::x
    led.coordinates[1] = b.get (i + 2).asDouble ();// VZ::y -> icub::y
    led.coordinates[2] = -b.get (i + 3).asDouble (); // VZ::-z -> icub::z

//    cout << led.channelName << " " << led.coordinates[0] << " " << led.coordinates[1] << " " << led.coordinates[2]
//        << endl;

    if (led.channelName == CHANNEL201.c_str () || led.channelName == CHANNEL202.c_str () || led.channelName
        == CHANNEL203.c_str () || led.channelName == CHANNEL204.c_str ())
    {
      if (fabs (led.coordinates[0]) < 0.001 && fabs (led.coordinates[1]) < 0.001 && fabs (led.coordinates[2]) < 0.001)
      {
        std::cout << "led " << led.channelName << "cannot be seen" << std::endl;
        return false;
      }
    }

    //convert to icub's origin
    led.coordinates[0] += 0.38;
    led.coordinates[1] -= 0.095;
    led.coordinates[2] -= 0.0018;

//    cout << led.channelName << " " << led.coordinates[0] << " " << led.coordinates[1] << " " << led.coordinates[2]
//        << endl;
    vectorList.push_back (led);
  }
  return true;
}

bool
getTransformation (tf::Transform &transform)
{
  std::vector<bool> leds_set (4, false);
  Vector led201, led202, led203, led204;
  if (getCoordinatesByChannelName (CHANNEL201.c_str (), led201))
    leds_set[0] = true;
  if (getCoordinatesByChannelName (CHANNEL202.c_str (), led202))
    leds_set[1] = true;
  if (getCoordinatesByChannelName (CHANNEL203.c_str (), led203))
    leds_set[2] = true;
  if (getCoordinatesByChannelName (CHANNEL204.c_str (), led204))
    leds_set[3] = true;

  if (leds_set[0] && leds_set[1] && leds_set[2] && leds_set[3])
  {
    //    std::cout<<"all leds are cool"<<std::endl;
    Vector front_mid = (led201 + led202) / 2.0;
    Vector rear_mid = (led203 + led204) / 2.0;
    Vector delta_mid = front_mid - rear_mid;
    //    Vector forward = front_mid + delta_mid;

    tf::Vector3 hat_arrow_origin;
    hat_arrow_origin.setX (front_mid[0]);
    hat_arrow_origin.setY (front_mid[1]);
    hat_arrow_origin.setZ (front_mid[2]);

    tf::Vector3 hat_x_axis;
    hat_x_axis.setX (delta_mid[0]);
    hat_x_axis.setY (delta_mid[1]);
    hat_x_axis.setZ (delta_mid[2]);
    hat_x_axis.normalize ();

    tf::Vector3 hat_y_axis;
    hat_y_axis.setX (led202[0]);
    hat_y_axis.setY (led202[1]);
    hat_y_axis.setZ (led202[2]);
    hat_y_axis -= hat_arrow_origin;
    hat_y_axis.normalize ();

    tf::Vector3 hat_z_axis;
    hat_z_axis = hat_x_axis.cross (hat_y_axis);

    //A frame is the base_footprint link
    tf::Transform A;
    A.setOrigin (btVector3 (0, 0, 0));
    A.setRotation (btQuaternion (btVector3 (0, 0, 1), 0));

    btMatrix3x3 hat_frame_rot;
    hat_frame_rot.setValue (hat_x_axis.x (), hat_y_axis.x (), hat_z_axis.x (), hat_x_axis.y (), hat_y_axis.y (),
                            hat_z_axis.y (), hat_x_axis.z (), hat_y_axis.z (), hat_z_axis.z ());

    tf::Transform hat_frame;
    hat_frame.setBasis (hat_frame_rot);
    hat_frame.setOrigin (hat_arrow_origin);

    transform.mult (hat_frame, A.inverse ());
    if(calibrated)
    {
		std::cout << hat_frame.getOrigin ().x () << " " << hat_frame.getOrigin ().y () << " "
        << hat_frame.getOrigin ().z () << "\n";
	}
    

    btScalar roll, pitch, yaw;
    hat_frame_rot.getRPY (roll, pitch, yaw);

	if(calibrated)
    {
		std::cout << "roll: " << roll * 180 / M_PI << std::endl;
		std::cout << "pitch: " << pitch * 180 / M_PI << std::endl;
		std::cout << "yaw: " << yaw * 180 / M_PI << std::endl;
	}
    
    if(calibrated)
    {
		 hat_frame_rot.setRPY (roll - roll_offset, pitch - pitch_offset, yaw - yaw_offset);
		 hat_frame.setBasis (hat_frame_rot);
	}
    transform = hat_frame;

    //    std::cout << "yaw: " << tf::getYaw (hat_frame.getRotation ()) * 180 / M_PI << std::endl;

    //    std::cout << hat_frame.get << " " << hat_frame.getOrigin ().y () << " "
    //        << hat_frame.getOrigin ().z () << "\n";

    //    offset.resize (3);
    //    offset[0] = hat_frame.getOrigin ().x ();
    //    offset[1] = hat_frame.getOrigin ().y ();
    //    offset[2] = hat_frame.getOrigin ().z ();
    //
    //    rpy.resize (3);
    //    hat_frame.getBasis ().getRPY (rpy[0], rpy[1], rpy[2]);
    return true;
  }
  else
  {
    //    std::cout << "at least one led is not visible to the mocap" << std::endl;
    return false;
  }
}

//tf::Transform
//getTF (const Vector& offset, const Vector& rpy)
//{
//  tf::Transform transform_hat;
//  transform_hat.setOrigin (tf::Vector3 (offset[0], offset[1], offset[2]));
//  transform_hat.setRotation (tf::createQuaternionFromRPY (rpy[0], rpy[1], rpy[2]));
//
//  return transform_hat;
//}
//
//bool
//calcTF (Vector& offset, Vector& rpy, tf::Transform& transform)
//{
//  if (getTransformation (offset, rpy))
//  {
//    transform = getTF (offset, rpy);
//    return true;
//  }
//  else
//    return false;
//}
