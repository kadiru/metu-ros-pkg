/*
 * SSLRealVision.cpp
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
 * IEEE METU Robotics and Automation Society (IEEE METU RAS)
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
 *     * Neither the name of IEEE METU RAS nor the names of its
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

#include "ssl_vision/SSLRealVision.h"

namespace ssl {

SSLRealVision::SSLRealVision(ros::NodeHandle* nh):SSLVision(nh) {

  client_.open (true);

  init();
}

SSLRealVision::~SSLRealVision() {
	// TODO Auto-generated destructor stub
}

void
SSLRealVision::update ()
{
  //TODO: camera merging has not been done yet
  if (client_.receive (packet_))
  {
    ROS_DEBUG("packet received");

    if (packet_.has_detection ())
    {
      SSL_DetectionFrame detection = packet_.detection ();
      global_state_.header.stamp = ros::Time::now();
      global_state_.header.frame_id = ssl::naming::frame::FIELD;

      int balls_n = detection.balls_size ();
      int robots_blue_n = detection.robots_blue_size ();
      int robots_yellow_n = detection.robots_yellow_size ();

//      std::cout<<balls_n<<std::endl;
//      std::cout<<robots_blue_n<<std::endl;
//      std::cout<<robots_yellow_n<<std::endl;

      //Ball info:
      global_state_.balls.clear();
      for (int i = 0; i < balls_n; i++)
      {
        SSL_DetectionBall ball = detection.balls (i);
        printBallInfo(ball);
        ssl_msgs::BallState ball_state;
        cvtToBallState(ball_state, ball);
        global_state_.balls.push_back(ball_state);
      }

      //Blue robot info:
      //TODO: try to find the most probable id for an unidentified robot
      //uint8_t suspicious_index = -1;
      for (int i = 0; i < robots_blue_n; i++)
      {
        SSL_DetectionRobot robot = detection.robots_blue (i);
        printf ("-Robot(B) (%2d/%2d): ", i + 1, robots_blue_n);
        printRobotInfo (robot);

        ssl_msgs::GlobalRobotState robot_state;
        if(cvtToRobotState(robot_state, robot)){}
          global_state_.blue_team[robot_state.id] = robot_state;
      }
      //if(suspicious_index!= -1){}


      //Yellow robot info:
      for (int i = 0; i < robots_yellow_n; i++)
      {
        SSL_DetectionRobot robot = detection.robots_yellow (i);
        printf ("-Robot(Y) (%2d/%2d): ", i + 1, robots_yellow_n);
        printRobotInfo (robot);
        ssl_msgs::GlobalRobotState robot_state;
        if(cvtToRobotState(robot_state, robot))
          global_state_.yellow_team[robot_state.id] = robot_state;
      }

    }
    //see if packet contains geometry data:
    if (packet_.has_geometry ())
    {
      const SSL_GeometryData & geom = packet_.geometry ();
      field_width_ = geom.field().field_width();
      field_height_= geom.field().field_length();
      field_outer_width_ = geom.field().field_width() + 2* geom.field().boundary_width();
      field_outer_height_= geom.field().field_length()+ 2* geom.field().boundary_width();

      //update parameters
      nh_->setParam("Field/width",field_width_);
      nh_->setParam("Field/height",field_height_);
      nh_->setParam("Field/outer_width",field_outer_width_);
      nh_->setParam("Field/outer_height",field_outer_height_);

      printGeomInfo (geom);
    }
  }
}

void
SSLRealVision::cvtToBallState(ssl_msgs::BallState& ball_state, const SSL_DetectionBall& ball)
{
  if(ball.has_area())
    ball_state.area = ball.area();
  if(ball.has_confidence())
    ball_state.confidence = ball.confidence();
  if(ball.has_pixel_x())
    ball_state.pix_coord.x = ball.pixel_x();
  if(ball.has_pixel_y())
    ball_state.pix_coord.y = ball.pixel_y();
  if(ball.has_x())
    ball_state.position.x = ball.x();
  if (ball.has_y())
    ball_state.position.y = ball.y();
  if (ball.has_z())
    ball_state.position.z = ball.z();
}

bool
SSLRealVision::cvtToRobotState(ssl_msgs::GlobalRobotState& robot_state, const SSL_DetectionRobot & robot)
{
  if(robot.has_confidence())
     robot_state.confidence = robot.confidence();
   if(robot.has_height())
     robot_state.height = robot.height();
   if(robot.has_orientation())
     robot_state.pose.theta = robot.orientation();
   if(robot.has_pixel_x())
     robot_state.pix_coord.x = robot.pixel_x();
   if(robot.has_pixel_y())
     robot_state.pix_coord.y = robot.pixel_y();
   if(robot.has_x())
     robot_state.pose.x = robot.x();
   if(robot.has_y())
     robot_state.pose.y = robot.y();
   if(robot.has_robot_id())
   {
     robot_state.id = robot.robot_id();
     return true;
   }
   return false;
}

void
SSLRealVision::printRobotInfo (const SSL_DetectionRobot & robot)
{
  printf ("CONF=%4.2f ", robot.confidence ());
  if (robot.has_robot_id ())
  {
    printf ("ID=%3d ", robot.robot_id ());
  }
  else
  {
    printf ("ID=N/A ");
  }
  printf (" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ", robot.height (), robot.x (), robot.y ());
  if (robot.has_orientation ())
  {
    printf ("ANGLE=%6.3f ", robot.orientation ());
  }
  else
  {
    printf ("ANGLE=N/A    ");
  }
  printf ("RAW=<%8.2f,%8.2f>\n", robot.pixel_x (), robot.pixel_y ());
}

void
SSLRealVision::printGeomInfo (const SSL_GeometryData& geom)
{
  printf ("-[Geometry Data]-------\n");

  const SSL_GeometryFieldSize & field = geom.field ();
  printf ("Field Dimensions:\n");
  printf ("  -line_width=%d (mm)\n", field.line_width ());
  printf ("  -field_length=%d (mm)\n", field.field_length ());
  printf ("  -field_width=%d (mm)\n", field.field_width ());
  printf ("  -boundary_width=%d (mm)\n", field.boundary_width ());
  printf ("  -referee_width=%d (mm)\n", field.referee_width ());
  printf ("  -goal_width=%d (mm)\n", field.goal_width ());
  printf ("  -goal_depth=%d (mm)\n", field.goal_depth ());
  printf ("  -goal_wall_width=%d (mm)\n", field.goal_wall_width ());
  printf ("  -center_circle_radius=%d (mm)\n", field.center_circle_radius ());
  printf ("  -defense_radius=%d (mm)\n", field.defense_radius ());
  printf ("  -defense_stretch=%d (mm)\n", field.defense_stretch ());
  printf ("  -free_kick_from_defense_dist=%d (mm)\n", field.free_kick_from_defense_dist ());
  printf ("  -penalty_spot_from_field_line_dist=%d (mm)\n", field.penalty_spot_from_field_line_dist ());
  printf ("  -penalty_line_from_spot_dist=%d (mm)\n", field.penalty_line_from_spot_dist ());

  int calib_n = geom.calib_size ();
  for (int i = 0; i < calib_n; i++)
  {
    const SSL_GeometryCameraCalibration & calib = geom.calib (i);
    printf ("Camera Geometry for Camera ID %d:\n", calib.camera_id ());
    printf ("  -focal_length=%.2f\n", calib.focal_length ());
    printf ("  -principal_point_x=%.2f\n", calib.principal_point_x ());
    printf ("  -principal_point_y=%.2f\n", calib.principal_point_y ());
    printf ("  -distortion=%.2f\n", calib.distortion ());
    printf ("  -q0=%.2f\n", calib.q0 ());
    printf ("  -q1=%.2f\n", calib.q1 ());
    printf ("  -q2=%.2f\n", calib.q2 ());
    printf ("  -q3=%.2f\n", calib.q3 ());
    printf ("  -tx=%.2f\n", calib.tx ());
    printf ("  -ty=%.2f\n", calib.ty ());
    printf ("  -tz=%.2f\n", calib.tz ());

    if (calib.has_derived_camera_world_tx () && calib.has_derived_camera_world_ty ()
        && calib.has_derived_camera_world_tz ())
    {
      printf ("  -derived_camera_world_tx=%.f\n", calib.derived_camera_world_tx ());
      printf ("  -derived_camera_world_ty=%.f\n", calib.derived_camera_world_ty ());
      printf ("  -derived_camera_world_tz=%.f\n", calib.derived_camera_world_tz ());
    }
  }
}

void
SSLRealVision::printBallInfo (const SSL_DetectionBall& ball)
{
  printf ("-Ball : CONF=%4.2f POS=<%9.2f,%9.2f> ", ball.confidence (), ball.x (), ball.y ());
  if (ball.has_z ())
  {
    printf ("Z=%7.2f ", ball.z ());
  }
  else
  {
    printf ("Z=N/A   ");
  }
  printf ("RAW=<%8.2f,%8.2f>\n", ball.pixel_x (), ball.pixel_y ());
}

}
