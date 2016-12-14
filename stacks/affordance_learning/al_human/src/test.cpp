/*
 * test.cpp
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

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "tf/transform_listener.h"
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

#include "al_utils/al_utils.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "wiimote/State.h"

const std::string BASE_FOOTPRINT_FRAME = "base_footprint";
const std::string OPENNI_DEPTH_FRAME = "camera_depth_frame";
const std::string LEFT_SHOULDER_FRAME = "left_shoulder";
const std::string RIGHT_SHOULDER_FRAME = "right_shoulder";
const std::string LEFT_ELBOW_FRAME = "left_elbow";
const std::string RIGHT_ELBOW_FRAME = "right_elbow";
const std::string NECK_FRAME = "neck";
const std::string TORSO_FRAME = "torso";
const std::string HEAD_FRAME = "head";

ros::NodeHandle* nh_;
tf::TransformListener* listener_;
ros::Subscriber sub_imu_data_;
ros::Subscriber sub_wiimote_state_;
sensor_msgs::Imu imu_data_;
robot_state_publisher::RobotStatePublisher* pub_human_state_;

std::map<std::string, double> joint_positions_;
std::map<std::string, double> prev_joint_positions_;
std::map<std::string, tf::StampedTransform> frame_transforms_;
float r_hand_gripper_pos_ = 0;
bool first_run_ = true;
KDL::Tree* tree_human_;

void
update (std::map<std::string, tf::StampedTransform>& frame_transforms);

bool
updateTransforms (std::map<std::string, tf::StampedTransform>& frame_transforms);

bool
updateJointStates (const std::map<std::string, tf::StampedTransform>& frame_transforms,
                   std::map<std::string, double> joint_positions);

//assumes torso related frames are not ill-transformed
//void
//extractTorsoJointsPositions (std::map<std::string, tf::StampedTransform>& frame_transforms, double& roll,
//                             double& pitch, double& yaw);

void
imuDataCallback (sensor_msgs::Imu::ConstPtr imu_data);

void
wiimoteStateCallback (wiimote::State::ConstPtr wiimote_state);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "test_state_pub");

  nh_ = new ros::NodeHandle ();
  listener_ = new tf::TransformListener ();
  tree_human_ = new KDL::Tree ();

  std::string robot_desc_string;
  nh_->param ("test_description", robot_desc_string, std::string ());
  if (!kdl_parser::treeFromString (robot_desc_string, *tree_human_))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }
  pub_human_state_ = new robot_state_publisher::RobotStatePublisher (*tree_human_);

  std::vector<std::string> joint_names;
  gazebo_msgs::SetModelConfiguration srv_set_joints;
  std::map<std::string, double>::iterator it;
  //wait for a healthy transformation set is obtained
  ros::Rate r (1.0);

  std::string link_joint = "link_joint";
  srv_set_joints.request.joint_names.push_back (link_joint);
  srv_set_joints.request.joint_positions.resize (1);
  srv_set_joints.request.model_name = "single_joint";
  srv_set_joints.request.urdf_param_name = "test_description";

  while (nh_->ok ())
  {
    float angle = al::math::fRand (-M_PI, M_PI);
    std::cout << "angle: " << angle << "\n";
    joint_positions_[link_joint] = angle;
    srv_set_joints.request.joint_positions[0] = joint_positions_[link_joint];

    pub_human_state_->publishTransforms (joint_positions_, ros::Time::now ());
    ros::spinOnce ();
    if (!ros::service::call ("/gazebo/set_model_configuration", srv_set_joints))
      ROS_WARN("something wrong with set_model_configuration service");
    else
    {
      if (srv_set_joints.response.success)
        ROS_INFO("model set successfully");
      else
        ROS_INFO("model cannot be set successfully");
      std::cout << srv_set_joints.response.status_message << std::endl;
    }

    ros::spinOnce ();
    r.sleep ();
  }

  return 0;
}

void
imuDataCallback (sensor_msgs::Imu::ConstPtr imu_data)
{
  imu_data_ = *imu_data;
}

void
wiimoteStateCallback (wiimote::State::ConstPtr wiimote_state)
{
  if (wiimote_state->buttons[6])
  {
    std::cout << "close the gripper\n";
    if (r_hand_gripper_pos_ >= 0.01)
      r_hand_gripper_pos_ -= 0.01;
  }

  if (wiimote_state->buttons[7])
  {
    std::cout << "open the gripper\n";
    if (r_hand_gripper_pos_ < 5.5)
      r_hand_gripper_pos_ += 0.01;
  }

}

bool
updateTransforms (std::map<std::string, tf::StampedTransform>& frame_transforms)
{
  bool user_found = false;
  int max_n_user = 10;
  int user_cnt = 1;
  std::string torso_frame_name;

  tf::StampedTransform tf_wrt_base_footprint;
  while (!user_found && user_cnt < max_n_user && nh_->ok ())
  {
    torso_frame_name = "torso_" + al::facilities::toString (user_cnt);
    try
    {
      listener_->lookupTransform (BASE_FOOTPRINT_FRAME.c_str (), torso_frame_name, ros::Time (0), tf_wrt_base_footprint);
      frame_transforms[TORSO_FRAME] = tf_wrt_base_footprint;

      user_found = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN ("no such transform to %s", torso_frame_name.c_str());
      user_cnt++;
    }
  }
  if (!user_found)
  {
    ROS_ERROR ("there is no user in the environment");
    //TODO: assume previous joint states same for 1 second, otherwise return false
    return false;
  }

  std::string r_sho_frame_name = "left_shoulder_" + al::facilities::toString (user_cnt);
  try
  {
    listener_->lookupTransform (BASE_FOOTPRINT_FRAME.c_str (), r_sho_frame_name, ros::Time (0), tf_wrt_base_footprint);
    frame_transforms[RIGHT_SHOULDER_FRAME] = tf_wrt_base_footprint;
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN ("no such transform to %s", r_sho_frame_name.c_str());
    //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
    return false;
  }

  std::string r_elb_frame_name = "left_elbow_" + al::facilities::toString (user_cnt);
  try
  {
    listener_->lookupTransform (BASE_FOOTPRINT_FRAME.c_str (), r_elb_frame_name, ros::Time (0), tf_wrt_base_footprint);
    frame_transforms[RIGHT_ELBOW_FRAME] = tf_wrt_base_footprint;
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN ("no such transform to %s", r_elb_frame_name.c_str());
    //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
    return false;
  }
  /*
   //no need to get left_hand_ because it is just a shifted version of left_elbow_
   std::string l_sho_frame_name = "right_shoulder_" + al::facilities::toString (user_cnt);
   try
   {
   listener_->lookupTransform (BASE_FOOTPRINT_FRAME.c_str (), l_sho_frame_name, ros::Time (0), tf_wrt_base_footprint);
   frame_transforms[LEFT_SHOULDER_FRAME] = tf_wrt_base_footprint;
   }
   catch (tf::TransformException ex)
   {
   ROS_WARN ("no such transform to %s", l_sho_frame_name.c_str());
   //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
   return false;
   }

   std::string l_elb_frame_name = "right_elbow_" + al::facilities::toString (user_cnt);
   try
   {
   listener_->lookupTransform (BASE_FOOTPRINT_FRAME.c_str (), l_elb_frame_name, ros::Time (0), tf_wrt_base_footprint);
   frame_transforms[LEFT_ELBOW_FRAME] = tf_wrt_base_footprint;
   }
   catch (tf::TransformException ex)
   {
   ROS_WARN ("no such transform to %s", l_elb_frame_name.c_str());
   //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
   return false;
   }
   */
  //no need to get left_hand_ because it is just a shifted version of left_elbow_
  return true;
}

bool
updateJointStates (const std::map<std::string, tf::StampedTransform>& frame_transforms,
                   std::map<std::string, double> joint_positions)
{

  tf::Vector3 tmp_x (0, 1, 0);
  tf::Vector3 tmp_y (0, 0, 1);
  tf::Vector3 tmp_z (1, 0, 0);
  btMatrix3x3 home_rot;
  home_rot.setValue (tmp_x.x (), tmp_y.x (), tmp_z.x (), tmp_x.y (), tmp_y.y (), tmp_z.y (), tmp_x.z (), tmp_y.z (),
                     tmp_z.z ());

  tf::Vector3 torso_translation = home_rot.transpose () * frame_transforms_[TORSO_FRAME].getOrigin ();
  btMatrix3x3 torso_rotation = home_rot.transpose () * frame_transforms_[TORSO_FRAME].getBasis ();
  double roll, pitch, yaw;
  torso_rotation.getEulerYPR (yaw, pitch, roll, 1);//angles are around ZYX. sequence is YXZ

  //  btMatrix3x3 r_z (btQuaternion (tf::Vector3 (0, 0, 1), yaw));
  //  btMatrix3x3 r_y (btQuaternion (tf::Vector3 (0, 1, 0), pitch));
  //  btMatrix3x3 r_x (btQuaternion (tf::Vector3 (1, 0, 0), roll));
  //  btMatrix3x3 rot = r_y * r_x * r_z;

  joint_positions_["base_footprint_to_human_footprint"] = 0;
  joint_positions_["human_footprint_to_torso_x"] = torso_translation.x ();
  joint_positions_["torso_x_to_torso_y"] = torso_translation.y ();
  joint_positions_["torso_y_to_torso_z"] = torso_translation.z ();
  joint_positions_["torso_z_to_pitch"] = pitch;
  joint_positions_["torso_pitch_to_roll"] = roll;
  joint_positions_["torso_roll_to_yaw"] = yaw;

  //  tf::Transform transform_sho;
  //  btMatrix3x3 sho_rotation = frame_transforms_[TORSO_FRAME].getBasis ().transpose ()
  //      * frame_transforms_[RIGHT_SHOULDER_FRAME].getBasis ();
  //
  //  sho_rotation.getEulerYPR (yaw, pitch, roll, 1);//order of angles ZYX, order of rotation -euler- YXZ.
  //  std::cout << "+1: y: " << pitch * 180 / M_PI << "\tx: " << roll * 180 / M_PI << "\tz: " << yaw * 180 / M_PI << "\n";

  //  btMatrix3x3 shoulder_ = sho_rotation * frame_transforms_[TORSO_FRAME].getBasis ();
  //  btMatrix3x3 shoulder = r_z * r_y * r_x * frame_transforms_[TORSO_FRAME].getBasis ();
  //
  //  tf::StampedTransform transform;
  //
  //  //  transform.frame_id_ = "r_shoulder_home";
  //  transform.frame_id_ = "torso_1";
  //  transform.child_frame_id_ = "r_shoulder_final";
  //
  //  transform.stamp_ = ros::Time::now ();
  //  transform.setOrigin (tf::Vector3 (0, 0, 0));
  //  transform.setBasis (sho_rotation);
  //  broadcaster_->sendTransform (transform);

  //  joint_positions_["torso_yaw_to_r_shoulder_home"] = 0;
  //  joint_positions_["r_shoulder_home_to_pitch"] = pitch;
  //  joint_positions_["r_shoulder_pitch_to_roll"] = roll;
  //  joint_positions_["r_shoulder_roll_to_yaw"] = yaw;
  //
  //  btMatrix3x3 elb_rotation = frame_transforms_[RIGHT_SHOULDER_FRAME].getBasis ().transpose ()
  //      * frame_transforms_[RIGHT_ELBOW_FRAME].getBasis ();
  /*
   //first method for elbow angle extraction -> 1DOF
   btQuaternion q;
   elb_rotation.getRotation (q);

   joint_positions_["r_shoulder_yaw_to_r_upper_arm"] = 0;
   joint_positions_["r_upper_arm_to_r_elbow_home"] = 0;
   if (fabs (q.getAxis ().y ()) > 0.90)
   joint_positions_["r_elbow_home_to_pitch"] = q.getAngle () * al::math::sgn (q.getAxis ().y ());
   else
   //singularity, assume 0 degrees
   joint_positions_["r_elbow_home_to_pitch"] = 0;

   joint_positions_["r_elbow_pitch_to_r_lower_arm"] = 0;*/

  //second method for elbow angle extraction-> 3DOF (not so real but it compansates the error)

  //  joint_positions_["r_elbow_pitch_to_r_hand_home"] = 0;
  //
  //  joint_positions_["r_hand_home_to_yaw"] = 0;
  //  joint_positions_["r_hand_yaw_to_pitch"] = 0;
  //  joint_positions_["r_hand_pitch_to_roll"] = 0;

  //  std::cout << " x: " << std::setw (8) << std::setprecision (3) << q.getAxis ().x () << "\ty: " << std::setw (8)
  //      << std::setprecision (3) << q.getAxis ().y () << "\tz: " << q.getAxis ().z () << "\t angle: " << std::setw (8)
  //      << std::setprecision (3) << q.getAngle () * 180 / M_PI << "\n";

  //  for (uint i = 0; i < 3; i++)
  //    std::cout << elb_rotation.getRow (i).x () << " " << elb_rotation.getRow (i).y () << " "
  //        << elb_rotation.getRow (i).z () << "\n";
  //  std::cout << "\n";

  //This doesn't work!
  //  tf::Transform t;
  //  t = frame_transforms_[RIGHT_ELBOW_FRAME] * frame_transforms_[RIGHT_SHOULDER_FRAME].inverse ();
  //  btMatrix3x3 r = t.getBasis ();
  //
  //  for (uint i = 0; i < 3; i++)
  //    std::cout << r.getRow (i).x () << " " << r.getRow (i).y () << " " << r.getRow (i).z () << "\n";
  //  std::cout << "\n";

  //  tf::Vector3 x_prime = elb_rotation.getColumn (0);
  //  tf::Vector3 x (1, 0, 0);
  //  float elbow_angle = fabs (al::math::getAngleBetween (x, x_prime));
  //  elbow_angle *= al::math::sgn (x.cross (x_prime).getY ());

  //  try
  //  {
  //    tf::StampedTransform tmp;
  //    listener_->lookupTransform ("left_shoulder_1", "left_elbow_1", ros::Time (0), tmp);
  //
  //    std::cout << "*relative: " << std::endl;
  //    for (uint i = 0; i < 3; i++)
  //      std::cout << tmp.getBasis ().getRow (i).x () << " " << tmp.getBasis ().getRow (i).y () << " "
  //          << tmp.getBasis ().getRow (i).z () << "\n";
  //    std::cout << "\n";
  //  }
  //  catch (tf::TransformException ex)
  //  {
  //    ROS_WARN ("no such transform to %s", "left_elbow_1");
  //    //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
  //    return false;
  //  }


  /*
   transform_sho = frame_transforms_[LEFT_SHOULDER_FRAME] * frame_transforms_[TORSO_FRAME].inverse ();
   sho_rotation = transform_sho.getBasis ();
   sho_rotation.getEulerYPR (yaw, pitch, roll, 1);
   sho_rotation.getEulerYPR (yaw2, pitch2, roll2, 2);
   //  std::cout << "sol 2: " << "z: " << (int)(yaw2 * 180 / M_PI) << "\t" << "y: " << (int)(pitch2 * 180 / M_PI) << "\t"
   //      << "x: " << (int)(roll2 * 180 / M_PI) << "\n";

   if (yaw2 < 0)
   yaw2 += 2 * M_PI;

   if (pitch2 < 0)
   pitch2 += 2 * M_PI;

   if (roll2 < 0)
   roll2 += 2 * M_PI;

   joint_positions_["torso_yaw_to_l_shoulder_home"] = 0;
   joint_positions_["l_shoulder_home_to_yaw"] = yaw2;
   joint_positions_["l_shoulder_yaw_to_pitch"] = pitch2;
   joint_positions_["l_shoulder_pitch_to_roll"] = roll2;

   joint_positions_["l_shoulder_roll_to_l_upper_arm"] = 0;

   elb_rotation = frame_transforms_[LEFT_SHOULDER_FRAME].getBasis ().transpose ()
   * frame_transforms_[LEFT_ELBOW_FRAME].getBasis ();

   x_prime = elb_rotation.getColumn (0);
   elbow_angle = fabs (al::math::getAngleBetween (x, x_prime));
   elbow_angle *= al::math::sgn (x.cross (x_prime).getY ());

   joint_positions_["l_shoulder_roll_to_l_elbow_home"] = 0;
   joint_positions_["l_elbow_home_to_pitch"] = elbow_angle;
   joint_positions_["l_elbow_pitch_to_l_lower_arm"] = 0;

   joint_positions_["l_elbow_pitch_to_l_hand_home"] = 0;

   joint_positions_["l_hand_home_to_yaw"] = 0;
   joint_positions_["l_hand_yaw_to_pitch"] = 0;
   joint_positions_["l_hand_pitch_to_roll"] = 0;
   */
  return true;
}

//void
//update (std::map<std::string, tf::StampedTransform>& frame_transforms)
//{
//  bool user_found = false;
//  int max_n_user = 10;
//  int user_cnt = 1;
//
//  tf::StampedTransform transform_torso;
//  tf::StampedTransform transform_head;
//  tf::StampedTransform transform_l_sh, transform_l_elb;
//  tf::StampedTransform transform_r_sho, elb_rotation;
//
//  std::string torso_frame_name;
//
//  while (!user_found && user_cnt < max_n_user && nh_->ok ())
//  {
//    torso_frame_name = "torso_" + al::facilities::toString (user_cnt);
//    try
//    {
//      listener_->lookupTransform ("base_footprint", torso_frame_name, ros::Time (0), transform_torso);
//      user_found = true;
//    }
//    catch (tf::TransformException ex)
//    {
//      ROS_WARN ("no such transform to %s", torso_frame_name.c_str());
//      user_cnt++;
//    }
//  }
//
//  if (!user_found)
//  {
//    ROS_ERROR ("there is no user in the environment");
//    //TODO: assume previous joint states same for 1 second, otherwise return false
//    return;
//  }
//  tf::Vector3 tmp_x (0, 1, 0);
//  tf::Vector3 tmp_y (0, 0, 1);
//  tf::Vector3 tmp_z (1, 0, 0);
//  btMatrix3x3 home_rot;
//  home_rot.setValue (tmp_x.x (), tmp_y.x (), tmp_z.x (), tmp_x.y (), tmp_y.y (), tmp_z.y (), tmp_x.z (), tmp_y.z (),
//                     tmp_z.z ());
//
//  //torso_rot = torso_rotation*home_rot
//  btMatrix3x3 torso_rotation = transform_torso.getBasis () * home_rot.inverse ();
//  double roll, pitch, yaw;
//  torso_rotation.getRPY (roll, pitch, yaw);
//
//  joint_positions_["base_footprint_to_torso_x"] = transform_torso.getOrigin ().x ();
//  joint_positions_["torso_x_to_torso_y"] = transform_torso.getOrigin ().y ();
//  joint_positions_["torso_y_to_torso_z"] = transform_torso.getOrigin ().z ();
//  joint_positions_["torso_z_to_home"] = 0;
//
//  joint_positions_["torso_home_to_roll"] = roll;
//  joint_positions_["torso_roll_to_pitch"] = pitch;
//  joint_positions_["torso_pitch_to_yaw"] = yaw;
//
//  std::string r_sho_frame_name = "left_shoulder_" + al::facilities::toString (user_cnt);
//  try
//  {
//    listener_->lookupTransform (torso_frame_name, r_sho_frame_name, ros::Time (0), transform_r_sho);
//  }
//  catch (tf::TransformException ex)
//  {
//    ROS_WARN ("no such transform to %s", r_sho_frame_name.c_str());
//    //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
//    return;
//  }
//
//  btMatrix3x3 r_sho_rotation = transform_r_sho.getBasis ();
//  r_sho_rotation.getEulerZYX (yaw, pitch, roll, 1);
//
//  double roll2, pitch2, yaw2;
//  r_sho_rotation.getEulerZYX (yaw2, pitch2, roll2, 2);
//
//  if (yaw2 < 0)
//    yaw2 += 2 * M_PI;
//
//  if (pitch2 < 0)
//    pitch2 += 2 * M_PI;
//
//  if (roll2 < 0)
//    roll2 += 2 * M_PI;
//
//  joint_positions_["torso_yaw_to_r_shoulder_home"] = 0;
//  joint_positions_["r_shoulder_home_to_yaw"] = yaw2;
//  joint_positions_["r_shoulder_yaw_to_pitch"] = pitch2;
//  joint_positions_["r_shoulder_pitch_to_roll"] = roll2;
//
//  joint_positions_["r_shoulder_roll_to_r_upper_arm"] = 0;
//
//  std::string r_elb_frame_name = "left_elbow_" + al::facilities::toString (user_cnt);
//  try
//  {
//    listener_->lookupTransform (r_sho_frame_name, r_elb_frame_name, ros::Time (0), transform_r_elb);
//  }
//  catch (tf::TransformException ex)
//  {
//    ROS_WARN ("no such transform to %s", r_elb_frame_name.c_str());
//    //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
//    return;
//  }
//  btMatrix3x3 r_elb_rotation = transform_r_elb.getBasis ();
//  btQuaternion q;
//  r_elb_rotation.getRotation (q);
//  r_elb_rotation.getEulerZYX (yaw, pitch, roll, 1);
//  r_elb_rotation.getEulerZYX (yaw2, pitch2, roll2, 2);
//  //  std::cout << "axis: " << q.getAxis ().x () << "\t" << q.getAxis ().y () << "\t" << q.getAxis ().z () << "\t angle:"
//  //      << q.getAngle () * 180 / M_PI << std::endl;
//  //  std::cout << "PITCH   : " << pitch * 180 / M_PI << std::endl;
//  //  std::cout << "PITCH2  : " << pitch2 << std::endl;
//
//  if (yaw2 < 0)
//    yaw2 += 2 * M_PI;
//
//  if (pitch2 < 0)
//    pitch2 += 2 * M_PI;
//
//  if (roll2 < 0)
//    roll2 += 2 * M_PI;
//
//  joint_positions_["r_shoulder_roll_to_r_elbow_home"] = 0;
//  joint_positions_["r_elbow_home_to_pitch"] = q.getAngle () * al::math::sgn (q.getAxis ().y ());
//  joint_positions_["r_elbow_pitch_to_r_lower_arm"] = 0;
//
//  joint_positions_["r_elbow_pitch_to_r_hand_home"] = 0;
//
//  //temporarily zeroed joints
//  joint_positions_["r_hand_home_to_yaw"] = 0;
//  joint_positions_["r_hand_yaw_to_pitch"] = 0;
//  joint_positions_["r_hand_pitch_to_roll"] = 0;
//
//  /*
//   //get acceleration values
//   tf::Vector3 acc;
//   tf::vector3MsgToTF (imu_data_.linear_acceleration, acc);
//   acc = acc * (-1);//since gravity is negative by nature
//   acc = acc.normalize ();
//
//   //wiimote_to_base_footprint normal transformation
//   tf::Vector3 minus_z_axial (0, 0, -1);
//   btMatrix3x3 base_to_wii_matrix = al::math::getTransformationBetween (minus_z_axial, acc);
//   tf::Transform base_to_wii_tf;
//   base_to_wii_tf.setBasis (base_to_wii_matrix);
//
//   tf::Vector3 z_axial (0, 0, 1);
//   tf::Vector3 wii_z_in_base = base_to_wii_tf * z_axial;
//   //  std::cout << acc.x () << "\t" << acc.y () << "\t" << acc.z () << std::endl;
//   //  std::cout << "* " << wii_z_in_base.x () << "\t" << wii_z_in_base.y () << "\t" << wii_z_in_base.z () << std::endl;
//
//   //double check
//   float a = al::math::getAngleBetween (acc, minus_z_axial);
//   tf::Transform t;
//   if (fabs (a) < 0.01)
//   {
//   t.setIdentity ();
//   }
//   else
//   {
//   tf::Vector3 v = acc.cross (minus_z_axial);
//   v = v.normalize ();
//   tf::Quaternion q_;
//   q_.setRotation (v, a);
//   t.setRotation (q_);
//   }
//   tf::Vector3 tmp = t * z_axial;
//   //  std::cout << "wii : " << tmp.x () << "\t" << tmp.y () << "\t" << tmp.z () << std::endl;
//
//   //hand_to_base_footprint normal transformation
//   //get y axis of hand in base_footprint (same as the elbow)
//   try
//   {
//   listener_->lookupTransform ("base_footprint", r_elb_frame_name, ros::Time (0), transform_r_elb);
//   }
//   catch (tf::TransformException ex)
//   {
//   ROS_WARN ("no such transform to %s", r_elb_frame_name.c_str());
//   //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
//   return;
//   }
//   btMatrix3x3 r1 = transform_r_elb.getBasis ();
//   tf::Vector3 hand_y_in_base = r1.getColumn (1);
//   //  std::cout << "hand: " << hand_y_in_base.x () << "\t" << hand_y_in_base.y () << "\t" << hand_y_in_base.z ()
//   //      << std::endl;
//   tf::Vector3 y_axial (0, 1, 0);
//   tf::Transform tf_;
//   tf_.setBasis (r1);
//   tf::Vector3 tmp_ = r1 * y_axial;
//   //  std::cout << "h  : " << tmp_.x () << "\t" << tmp_.y () << "\t" << tmp_.z () << std::endl;
//
//   tf::Vector3 final_vec = tmp_.cross (tmp);
//   float final_ang = al::math::getAngleBetween (tmp_, tmp);
//   tf::Vector3 mapped_final_vec = tf_.inverse () * final_vec;
//   //  std::cout << mapped_final_vec.x () << "\t" << mapped_final_vec.y () << "\t" << mapped_final_vec.z () << "\n";
//   //  std::cout << final_ang * 180 / M_PI << std::endl;
//
//   /*
//   tf::Vector3 rot_vec_hand_to_wii = wii_z_in_base.cross (hand_y_in_base);
//   float rot_ang_hand_to_wii = al::math::getAngleBetween (wii_z_in_base, hand_y_in_base);
//   //  tf::Vector3 y_axial_in_body;
//   //  y_axial_in_body.setX(y_axial.)
//
//   //just modify rotation vector so that it reprsents a rot_vec in hand frame
//   tf::Vector3 rot_vec_hand_to_wii_wrt_hand;
//   rot_vec_hand_to_wii_wrt_hand.setValue (rot_vec_hand_to_wii.y (), rot_vec_hand_to_wii.z (), rot_vec_hand_to_wii.x ());
//   */
//  /*
//   tf::Quaternion q_hand_to_wii;
//   q_hand_to_wii.setRotation (mapped_final_vec, final_ang);
//   btMatrix3x3 hand_to_wii_matrix;
//   hand_to_wii_matrix.setRotation (q_hand_to_wii);
//   btScalar wii_roll, wii_pitch, wii_yaw;
//   hand_to_wii_matrix.getEulerZYX (wii_yaw, wii_pitch, wii_roll);
//   std::cout << "roll: " << wii_roll * 180 / M_PI << "\tpitch:" << wii_pitch * 180 / M_PI << "\tyaw:" << wii_yaw * 180
//   / M_PI << "\n";
//
//   joint_positions_["r_hand_roll_to_r_l_gripper"] = r_hand_gripper_pos_;//wiimote button controlled
//   joint_positions_["r_l_tip_joint"] = 0;
//   joint_positions_["r_hand_roll_to_r_r_gripper"] = r_hand_gripper_pos_;//wiimote button controlled
//   joint_positions_["r_r_tip_joint"] = 0;
//
//   joint_positions_["r_hand_home_to_yaw"] = wii_yaw;
//   joint_positions_["r_hand_yaw_to_pitch"] = wii_pitch;
//   joint_positions_["r_hand_pitch_to_roll"] = wii_roll;
//   */
//  pub_human_state_->publishTransforms (joint_positions_, ros::Time::now ());
//
//  /*
//   std::string r_sho_frame_name = "left_shoulder_" + al::facilities::toString (user_cnt);
//   try
//   {
//   listener_->lookupTransform ("base_footprint", r_sho_frame_name, ros::Time (0), transform_r_sho);
//   }
//   catch (tf::TransformException ex)
//   {
//   ROS_WARN ("no such transform to %s", r_sho_frame_name.c_str());
//   //TODO: lost the user, suppose previous joint states same for 1 second, otherwise return false
//   return;
//   }
//
//   btMatrix3x3 r_sho_rotation = transform_r_sho.getBasis () * home_rot.inverse ();
//   tf::Transform r_sho_overall_rotation, r_sho_home_rotation;
//   r_sho_overall_rotation.setBasis (transform_r_sho.getBasis ());
//   r_sho_home_rotation.setBasis (home_rot);
//   tf::Transform rot;
//   rot.mult (r_sho_overall_rotation, r_sho_home_rotation.inverse ());
//   //  btQuaternion q;
//   //  //  r_sho_rotation.getRotation (q);
//   //  q = rot.getRotation ();
//   //  std::cout << "axis: " << q.getAxis ().x () << "\t" << q.getAxis ().y () << "\t" << q.getAxis ().z () << "\t angle:"
//   //      << q.getAngle () << std::endl;
//   //  r_sho_rotation.getRPY (roll, pitch, yaw);
//   //  r_sho_rotation.getEulerZYX (yaw, pitch, roll, 1);
//   rot.getBasis ().getEulerZYX (yaw, pitch, roll, 1);
//   if (yaw < 0)
//   yaw += 2 * M_PI;
//
//   if (pitch < 0)
//   pitch += 2 * M_PI;
//
//   if (roll < 0)
//   roll += 2 * M_PI;
//   //  std::cout << "sol 1: " << "z: " << (int)(yaw * 180 / M_PI) << "\t" << "y: " << (int)(pitch * 180 / M_PI) << "\t"
//   //      << "x: " << (int)(roll * 180 / M_PI) << "\n";
//
//   double roll2, pitch2, yaw2;
//   //  r_sho_rotation.getEulerZYX (yaw2, pitch2, roll2, 2);
//   rot.getBasis ().getEulerZYX (yaw2, pitch2, roll2, 2);
//   if (yaw2 < 0)
//   yaw2 += 2 * M_PI;
//
//   if (pitch2 < 0)
//   pitch2 += 2 * M_PI;
//
//   if (roll2 < 0)
//   roll2 += 2 * M_PI;
//   //  std::cout << "sol 2: " << "z: " << (int)(yaw2 * 180 / M_PI) << "\t" << "y: " << (int)(pitch2 * 180 / M_PI) << "\t"
//   //      << "x: " << (int)(roll2 * 180 / M_PI) << "\n";
//
//   std::cout << "sol 2: " << "z: " << (int)(yaw2 * 180 / M_PI) << "\n";
//
//   joint_positions_["torso_yaw_to_r_shoulder_home"] = 0;
//
//   if (first_run_)
//   {
//   first_run_ = false;
//   }
//
//   joint_positions_["r_shoulder_home_to_yaw"] = yaw2;
//   joint_positions_["r_shoulder_yaw_to_pitch"] = pitch2;
//   joint_positions_["r_shoulder_pitch_to_roll"] = roll2;*/
//
//  //  bool transformation_healthy = true;
//  //  try
//  //  {
//  //    listener_->lookupTransform ("base_footprint", "torso_1", ros::Time (0), transform_torso);
//  //  }
//  //  catch (tf::TransformException ex)
//  //  {
//  //    transformation_healthy = false;
//  //  }
//  //
//  //  try
//  //  {
//  //    listener_->lookupTransform ("base_footprint", "neck_1", ros::Time (0), transform_head);
//  //  }
//  //  catch (tf::TransformException ex)
//  //  {
//  //    transformation_healthy = false;
//  //  }
//  //
//  //  try
//  //  {
//  //    listener_->lookupTransform ("base_footprint", "left_shoulder_1", ros::Time (0), transform_l_sh);
//  //  }
//  //  catch (tf::TransformException ex)
//  //  {
//  //    transformation_healthy = false;
//  //  }
//  //
//  //  try
//  //  {
//  //    listener_->lookupTransform ("base_footprint", "right_shoulder_1", ros::Time (0), transform_r_sho);
//  //  }
//  //  catch (tf::TransformException ex)
//  //  {
//  //    transformation_healthy = false;
//  //  }
//  //
//  //  try
//  //  {
//  //    listener_->lookupTransform ("base_footprint", "left_elbow_1", ros::Time (0), transform_l_elb);
//  //  }
//  //  catch (tf::TransformException ex)
//  //  {
//  //    transformation_healthy = false;
//  //  }
//  //
//  //  try
//  //  {
//  //    listener_->lookupTransform ("base_footprint", "right_elbow_1", ros::Time (0), transform_r_elb);
//  //  }
//  //  catch (tf::TransformException ex)
//  //  {
//  //    transformation_healthy = false;
//  //  }
//  //
//  //  if (transformation_healthy)
//  //  {
//  //    joint_positions_["base_footprint_to_torso_x"] = transform_torso.getOrigin ().x ();
//  //    joint_positions_["torso_x_to_torso_y"] = transform_torso.getOrigin ().y ();
//  //    joint_positions_["torso_y_to_torso_z"] = transform_torso.getOrigin ().z ();
//  //
//  //    //    tf::Vector3 torso_to_neck = transform_head.getOrigin () - transform_torso.getOrigin ();
//  //    //    torso_to_neck = torso_to_neck.normalize ();
//  //    //    tf::Vector3 torso_z (0, 0, 1);
//  //    //    btMatrix3x3 rot_matrix = al::math::getTransformationBetween (torso_z, torso_to_neck);
//  //
//  //    tf::Vector3 torso_z = transform_head.getOrigin () - transform_torso.getOrigin ();
//  //    torso_z = torso_z.normalize ();
//  //    tf::Vector3 torso_y = transform_r_sho.getOrigin () - transform_l_sh.getOrigin ();
//  //    torso_y = torso_y.normalize ();
//  //    tf::Vector3 torso_x = torso_y.cross (torso_z);
//  //    torso_x = torso_x.normalize ();
//  //
//  //    btMatrix3x3 rot_matrix;
//  //    rot_matrix.setValue (torso_x.x (), torso_y.x (), torso_z.x (), torso_x.y (), torso_y.y (), torso_z.y (),
//  //                         torso_x.z (), torso_y.z (), torso_z.z ());
//  //
//  //    double roll, pitch, yaw;
//  //    rot_matrix.getRPY (roll, pitch, yaw);
//  //    joint_positions_["torso_z_to_torso_roll"] = roll;
//  //    joint_positions_["torso_roll_to_torso_pitch"] = pitch;
//  //    joint_positions_["torso_pitch_to_torso_yaw"] = yaw;
//  //
//  //    tf::Vector3 r_shoulder_z = transform_l_elb.getOrigin () - transform_l_sh.getOrigin ();
//  //    r_shoulder_z = r_shoulder_z.normalize ();
//  //    tf::Vector3 r_shoulder_y = transform_r_sho.getOrigin () - transform_l_sh.getOrigin ();
//  //    r_shoulder_y = r_shoulder_y.normalize ();
//  //    tf::Vector3 r_shoulder_x = r_shoulder_y.cross (r_shoulder_z);
//  //    r_shoulder_x = r_shoulder_x.normalize ();
//  //
//  //    joint_positions_["torso_yaw_to_neck"] = 0.0;
//  //    joint_positions_["neck_to_head"] = 0.0;
//  //    joint_positions_["torso_yaw_to_r_shoulder"] = 0.0;
//  //    joint_positions_["torso_yaw_to_l_shoulder"] = 0.0;
//  //
//  //    //    tf::Vector3 torso_to_neck = transform_head.getOrigin ();
//  //    //    torso_to_neck = torso_to_neck.normalize ();
//  //    //    tf::Vector3 torso_y (0, 1, 0);
//  //    //    tf::Vector3 rot_axis = torso_y.cross (torso_to_neck);TRANSO
//  //    //    rot_axis = rot_axis.normalize ();
//  //    //    float rot_angle = (torso_y.dot (torso_to_neck)) / (torso_y.length () * torso_to_neck.length ());
//  //    //    tf::Quaternion q;
//  //    //    q.setRotation (rot_axis, rot_angle);
//  //    //    btMatrix3x3 rotation;
//  //    //    rotation.setRotation (q);
//  //    //    double roll, pitch, yaw;
//  //    //    rotation.getRPY (roll, pitch, yaw);
//  //    //    joint_positions_["torso_z_to_torso_roll"] = roll;
//  //    //    joint_positions_["torso_roll_to_torso_pitch"] = pitch;
//  //    //    joint_positions_["torso_pitch_to_torso_yaw"] = yaw;
//  //
//  //    /*
//  //     btMatrix3x3 rotation;
//  //     tf::Quaternion q = transform_torso.getRotation ();
//  //     rotation.setRotation (q);
//  //     double roll, pitch, yaw;
//  //     rotation.getRPY (roll, pitch, yaw);
//  //
//  //     joint_positions_["torso_z_to_torso_roll"] = roll - M_PI_2;
//  //     //    joint_positions_["torso_roll_to_torso_pitch"] = pitch;
//  //     //    joint_positions_["torso_pitch_to_torso_yaw"] = yaw + M_PI_2;
//  //
//  //     //    joint_positions_["torso_z_to_torso_roll"] = roll;
//  //     joint_positions_["torso_roll_to_torso_pitch"] = pitch;
//  //     joint_positions_["torso_pitch_to_torso_yaw"] = yaw;
//  //     */
//  //  pub_human_state_->publishTransforms (joint_positions_, ros::Time::now ());
//  //  }
//  //  else
//  //  {
//  //    ROS_WARN("transformation is not healthy, publishing previous joint states");
//  //  }
//}
