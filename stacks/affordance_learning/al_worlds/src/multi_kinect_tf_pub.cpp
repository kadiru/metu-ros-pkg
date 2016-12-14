/*
 * multi_kinect_tf_pub.cpp
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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

const string CHANNEL101 = "Channel101";
const string CHANNEL102 = "Channel102";
const string CHANNEL103 = "Channel103";
const string CHANNEL104 = "Channel104";
const string CHANNEL105 = "Channel105";
const string CHANNEL106 = "Channel106";
const string CHANNEL107 = "Channel107";
const string CHANNEL108 = "Channel108";

//in the current configuration
//for kinect on the left of the robot
//**origin: 106
//**z_axis: 108
//**x_axis: 107
//for kinect on the right of the robot
//**origin: 101
//**z_axis: 103
//**x_axis: 104

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
bool first_data_received = false;
bool new_data_received = false;
Transform prev_transform;
Transform avg_tranform;
Transform avg_tranform_left;
Transform avg_tranform_right;
tf::StampedTransform* avg_tf = NULL;
tf::StampedTransform* avg_tf_left = NULL;
tf::StampedTransform* avg_tf_right = NULL;
bool tf_calculated = false;
bool left_active_ = true;
bool right_active_ = false;

//const int N_TRANSFORM = 10000;
const int N_TRANSFORM = 1000;
Vector
getCoordinatesByChannelName (const char* channelOfInterest);

bool
getCameraLedInfo (const Bottle& b);

void
getTransformation (Vector& offset, Vector& rpy, const std::string led_origin, const std::string led_x_axis,
                   const std::string led_z_axis);

tf::Transform
getTF (const Vector& offset, const Vector& rpy);

tf::Transform
calcTF (Vector& offset, Vector& rpy, const std::string led_origin, const std::string led_x_axis,
        const std::string led_z_axis);

//mocap data received callback
class DataPort : public BufferedPort<Bottle>
{
  virtual void
  onRead (Bottle& b)
  {
    static int data_count = 0;
    // process data in b
    bool safe_tf = getCameraLedInfo (b);
    if (data_count < N_TRANSFORM)
    {
      if (safe_tf)
      {
        //for left camera
        if (left_active_)
        {
          std::cout << "LEFT CAMERA" << std::endl;
          calcTF (offset, rpy, CHANNEL106, CHANNEL107, CHANNEL108);
          std::cout << "transformation :" << data_count << std::endl;
          std::cout << "translation: " << offset[0] << " " << offset[1] << " " << offset[2] << std::endl;
          std::cout << "orientation: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;
          for (uint8_t i = 0; i < offset.size (); i++)
            avg_tranform_left.offset[i] += offset[i] / N_TRANSFORM;
          for (uint8_t i = 0; i < rpy.size (); i++)
            avg_tranform_left.rpy[i] += rpy[i] / N_TRANSFORM;
        }

        if (right_active_)
        {
          //for right camera
          std::cout << "RIGHT CAMERA" << std::endl;
          calcTF (offset, rpy, CHANNEL101, CHANNEL103, CHANNEL104);
          std::cout << "transformation :" << data_count << std::endl;
          std::cout << "translation: " << offset[0] << " " << offset[1] << " " << offset[2] << std::endl;
          std::cout << "orientation: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;
          for (uint8_t i = 0; i < offset.size (); i++)
            avg_tranform_right.offset[i] += offset[i] / N_TRANSFORM;
          for (uint8_t i = 0; i < rpy.size (); i++)
            avg_tranform_right.rpy[i] += rpy[i] / N_TRANSFORM;
        }

        if (left_active_ || right_active_)
          data_count++;
      }
    }
    else
    {
      std::cout << "calculating average transform" << std::endl;
      if (left_active_)
      {
        std::cout << "LEFT CAMERA" << std::endl;
        avg_tf_left = new tf::StampedTransform (getTF (avg_tranform_left.offset, avg_tranform_left.rpy),
                                                ros::Time::now (), "/base_footprint", "/camera_link");
        std::cout << "translation: " << avg_tranform_left.offset[0] << " " << avg_tranform_left.offset[1] << " "
            << avg_tranform_left.offset[2] << std::endl;
        std::cout << "orientation: " << avg_tranform_left.rpy[0] << " " << avg_tranform_left.rpy[1] << " "
            << avg_tranform_left.rpy[2] << std::endl;
        tf_calculated = true;
      }

      if (right_active_)
      {
        std::cout << "RIGHT CAMERA" << std::endl;
        avg_tf_right = new tf::StampedTransform (getTF (avg_tranform_right.offset, avg_tranform_right.rpy),
                                                 ros::Time::now (), "/base_footprint", "/right_camera_link");
        std::cout << "translation: " << avg_tranform_right.offset[0] << " " << avg_tranform_right.offset[1] << " "
            << avg_tranform_right.offset[2] << std::endl;
        std::cout << "orientation: " << avg_tranform_right.rpy[0] << " " << avg_tranform_right.rpy[1] << " "
            << avg_tranform_right.rpy[2] << std::endl;
        tf_calculated = true;
      }

      this->close (); //close this connection
    }
  }
};

int
main (int argc, char* argv[])
{
  //ros initializations
  ros::init (argc, argv, "mocap_kinects");
  ros::NodeHandle n;

  //yarp initializations
  Network::init ();
  DataPort vzInPort;
  vzInPort.useCallback ();
  vzInPort.open ("/i:vzListen");
  Network::connect ("/vzRawPout", "/i:vzListen");

  std::cout << "waiting for mocap to be connected" << std::endl;
  while (!tf_calculated && n.ok ())
  {
  }
  std::cout << "tf is calculated" << std::endl;

  static tf::TransformBroadcaster br;

  ros::Rate r (50);
  while (n.ok ())
  {
    if (avg_tf_left != NULL)
    {
      br.sendTransform (tf::StampedTransform (*avg_tf_left, ros::Time::now (), "/base_footprint", "/left_camera_link"));
    }
    if (avg_tf_right != NULL)
    {
      br.sendTransform (
                        tf::StampedTransform (*avg_tf_right, ros::Time::now (), "/base_footprint", "/right_camera_link"));
    }
    ros::spinOnce ();
    r.sleep ();
  }
  Network::fini ();
  delete avg_tf_left;
  delete avg_tf_right;

  return 0;
}

Vector
getCoordinatesByChannelName (const char* channelOfInterest)
{
  for (uint i = 0; i < vectorList.size (); i++)
    if (vectorList[i].channelName == channelOfInterest)
      return vectorList[i].coordinates;

  cerr << "Could not find the queried channel of interest, something wrong with the leds or mocap, EXITING!" << endl;
  exit (-1);
}

bool
getCameraLedInfo (const Bottle& b)
{

  int channelCount = b.get (0).asInt ();//get number of active leds
  vectorList.clear ();
  for (int i = 1; i < channelCount * 5; i = i + 5)
  {
    LEDVector led;
    led.channelName = b.get (i).asString ();//channel name
    if (led.channelName == "Channel102" || led.channelName == "Channel105")
      continue;

    //now convert led coordinates from KDL frame back to the iCub frame
    /*
     led.coordinates[0] = -b.get (i + 3).asDouble ();//KDL's "-z" -> icub's "x"
     led.coordinates[1] = -b.get (i + 1).asDouble ();//KDL's "-x" -> icub's "y"
     led.coordinates[2] = b.get (i + 2).asDouble ();//KDL's "y" -> icub's "z"
     */

    //convert vz to icub frame
    led.coordinates[0] = -b.get (i + 1).asDouble ();// VZ::-x -> icub::x
    led.coordinates[1] = b.get (i + 2).asDouble ();// VZ::y -> icub::y
    led.coordinates[2] = -b.get (i + 3).asDouble (); // VZ::-z -> icub::z

    //    std::cout << led.channelName << led.coordinates[0] << " " << led.coordinates[1] << " " << led.coordinates[2]
    //        << std::endl;

    if (fabs (led.coordinates[0]) < 0.001 && fabs (led.coordinates[1]) < 0.001 && fabs (led.coordinates[2]) < 0.001)
      return false;

    led.coordinates[0] += 0.38;
    led.coordinates[1] -= 0.095;
    led.coordinates[2] -= 0.0018;

    //    cout << led.channelName << " " << led.coordinates[0] << " " << led.coordinates[1] << " " << led.coordinates[2]<< endl;
    vectorList.push_back (led);
  }
  return true;
}

void
getTransformation (Vector& offset, Vector& rpy, const std::string led_origin, const std::string led_x_axis,
                   const std::string led_z_axis)
{
  Vector ch209 = getCoordinatesByChannelName (led_origin.c_str ());
  //  Vector ch210 = getCoordinatesByChannelName (CHANNEL210);
  Vector ch211 = getCoordinatesByChannelName (led_x_axis.c_str ());
  Vector ch212 = getCoordinatesByChannelName (led_z_axis.c_str ());

  //get camera casis coordinate system

  //get origin:
  Vector cam_casis_origin = ch209;
  offset = cam_casis_origin;

  //get unit x vector
  Vector cam_casis_x = cam_casis_origin - ch212;
  double norm_cam_casis_x = norm (cam_casis_x);
  cam_casis_x = cam_casis_x / norm_cam_casis_x;

  //get unit xy plane vector
  //  Vector cam_casis_xy = (ch209 + ch210)/2 - cam_casis_origin;
  Vector cam_casis_xy = cam_casis_origin - ch211;
  double norm_cam_casis_xy = norm (cam_casis_xy);
  cam_casis_xy = cam_casis_xy / norm_cam_casis_xy;

  //get unit z vector
  Vector cam_casis_z = cross (cam_casis_x, cam_casis_xy);
  double norm_cam_casis_z = norm (cam_casis_z);
  cam_casis_z = cam_casis_z / norm_cam_casis_z;

  //get unit y vector
  Vector cam_casis_y = cross (cam_casis_z, cam_casis_x);
  double norm_cam_casis_y = norm (cam_casis_y);
  cam_casis_y = cam_casis_y / norm_cam_casis_y;

  //find rpy component
  Matrix rot (3, 3);

  Vector xG (3);
  xG[0] = 1;
  xG[1] = 0;
  xG[2] = 0;
  Vector yG (3);
  yG[0] = 0;
  yG[1] = 1;
  yG[2] = 0;
  Vector zG (3);
  zG[0] = 0;
  zG[1] = 0;
  zG[2] = 1;

  Vector v (3);
  v[0] = dot (cam_casis_x, xG);
  v[1] = dot (cam_casis_x, yG);
  v[2] = dot (cam_casis_x, zG);
  rot.setCol (0, v);

  v[0] = dot (cam_casis_y, xG);
  v[1] = dot (cam_casis_y, yG);
  v[2] = dot (cam_casis_y, zG);
  rot.setCol (1, v);

  v[0] = dot (cam_casis_z, xG);
  v[1] = dot (cam_casis_z, yG);
  v[2] = dot (cam_casis_z, zG);
  rot.setCol (2, v);

  rpy[0] = atan2 (rot (2, 1), rot (2, 2)); // r
  rpy[2] = atan2 (rot (1, 0), rot (0, 0)); // y
  rpy[1] = atan2 (-rot (2, 0), cos (rpy[2]) * rot (0, 0) + sin (rpy[2]) * rot (1, 0)); // p
}

tf::Transform
getTF (const Vector& offset, const Vector& rpy)
{
  tf::Transform icub_to_cam_top;
  icub_to_cam_top.setOrigin (tf::Vector3 (offset[0], offset[1], offset[2]));
  icub_to_cam_top.setRotation (tf::createQuaternionFromRPY (rpy[0], rpy[1], rpy[2]));

  tf::Transform cam_top_to_cam_focus;
  cam_top_to_cam_focus.setOrigin (tf::Vector3 (0.0, -0.026, 0.0));
  cam_top_to_cam_focus.setRotation (tf::createQuaternionFromRPY (0, 0, 0));

  //return the tf from icub to cam focus
  icub_to_cam_top.mult (icub_to_cam_top, cam_top_to_cam_focus);
  return icub_to_cam_top;
}

tf::Transform
calcTF (Vector& offset, Vector& rpy, const std::string led_origin, const std::string led_x_axis,
        const std::string led_z_axis)
{
  getTransformation (offset, rpy, led_origin, led_x_axis, led_z_axis);

  return getTF (offset, rpy);
}
