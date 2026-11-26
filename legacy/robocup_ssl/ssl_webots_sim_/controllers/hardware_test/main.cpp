/*  Copyright (C) 2011, Kadir Firat Uyanik 
 *    Middle East Technical University, Kovan Research Lab 
 *    kadir@ceng.metu.edu.tr 
 * 
 *    http://kovan.ceng.metu.edu.tr/~kadir 
 * 
 *  main.cpp is part of hardware_test. 
 * 
 *  hardware_test is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version. 
 * 
 *  hardware_test is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *  GNU General Public License for more details. 
 * 
 *  You should have received a copy of the GNU General Public License 
 *  along with hardware_test. If not, see <http://www.gnu.org/licenses/>. 
 */



#include "iostream"
#include "stdlib.h"
#include "string"
#include "vector"
#include "math.h"

#include "webots/Robot.hpp"
#include "webots/Servo.hpp"
#include "webots/DistanceSensor.hpp"

typedef unsigned int uint;

const uint TIME_STEP = 16;
const uint N_WHEELS = 3;
const int MAXINT = 100000;
const double PI = 3.141592;
const double R_WHEEL = 0.025;

template<typename T>
  int
  sign (T data)
  {
    if (data >= 0)
      return 1;
    else
      return -1;
  }

class SSLRobot : public webots::Robot
{
public:
  SSLRobot ();
  virtual
  ~SSLRobot ();
  void
  init ();

  void
  fini ();

  void
  run ();

  void
  move (float direction, float speed);

  void
  kick (float speed);

  void
  dribble (float speed, bool fwd_dir = true);

  bool hasBall();
private:
  std::vector<webots::Servo*> wheels_;
  webots::Servo* kicker_;
  webots::Servo* dribbler_;
  webots::DistanceSensor* ir_;
};

SSLRobot::~SSLRobot ()
{
}

SSLRobot::SSLRobot ()
{
  if (N_WHEELS == 3)
  {
    wheels_.push_back (getServo ("wheel0"));
    wheels_.push_back (getServo ("wheel1"));
    wheels_.push_back (getServo ("wheel2"));
  }
  else
    wheels_.push_back (getServo ("wheel3"));

  dribbler_ = getServo ("dribbler");
  kicker_ = getServo ("kicker");
//  ir_ = getDistanceSensor("ir");

  for (uint i = 0; i < N_WHEELS; i++)
    wheels_[i]->enablePosition (TIME_STEP);

  dribbler_->enablePosition (TIME_STEP);
  kicker_->enablePosition (TIME_STEP);
//  ir_->enable(TIME_STEP);
}

void
SSLRobot::init ()
{

}

void
SSLRobot::fini ()
{

}

void
SSLRobot::run ()
{
  uint step_cnt = 0;
  uint stage_cnt = -1;
  while (true)
  {
    if (step_cnt % 100 == 0)
    {
      step_cnt = 0;
      stage_cnt++;
    }
    if (stage_cnt % 8 == 0)
      stage_cnt = 0;
      
      double velocity_dir = 0;
      double velocity_mag = 0.5;
    //std::cout<<stage_cnt<<" ";
    switch (stage_cnt)
    {
      case 0:
      	velocity_dir = 0;
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;
        kick(0);
        dribble(0);
        break;
      case 1:
      	velocity_dir = PI/2;
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;
        kick(0);
        dribble(0);
        break;
      case 2:
      	velocity_dir = PI;	
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;        
        kick(0);
        dribble(0);
        break;
      case 3:
      	velocity_dir = -PI/2;
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;
        kick(0);
        dribble(0);
        break;
      case 4:
      	velocity_dir = PI/4;
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;
        kick(0);
        dribble(0);
        break;
      case 5:
      	velocity_dir = -PI/4;
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;
        kick(0);
        dribble(0);
        break;
      case 6:
      	velocity_dir = -3*PI/4;
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;
        kick(0);
        dribble(0);
        break;
      case 7:
      	velocity_dir = 3*PI/4;
        move (velocity_dir, velocity_mag);
        std::cout<<"velocity_direction: "<<velocity_dir*180/PI<< " degree"<<std::endl;
        kick(0);
        dribble(0);
        break;
      case 8:
        move(0,0);
        dribble(30);
        break;
      case 9:
        move(0,0);
        dribble(30,false);
        break;
      default:
        break;
    }

    if (step (TIME_STEP) == -1)
      break;

    step_cnt++;
  }
}

bool SSLRobot::hasBall()
{
//  if(ir_->getValue() == 1000)
//    return false;//no ball
//  else
//   return true;//it has ball
	return true;
}

void
SSLRobot::move (float direction, float speed)
{
  float v_x = speed * cos (direction);
  float v_y = speed * sin (direction);

  float wheel_vels[4];

  //three wheel kinematics
  if (N_WHEELS == 3)
  {
    //    wheel_vels[0] = 0;
    //    wheel_vels[1] = 0;
    //    wheel_vels[2] = -10;

    //    std::cout<<sign(wheel_vels[0])<<" "<<sign(wheel_vels[1])<<" "<<sign(wheel_vels[2])<<" "<<std::endl;


    wheel_vels[0] = (sin (60.0 * PI / 180.0) * v_x - cos (60.0 * PI / 180.0) * v_y) / R_WHEEL;
    wheel_vels[1] = v_y / R_WHEEL;
    wheel_vels[2] = (-sin (60.0 * PI / 180.0) * v_x - cos (60.0 * PI / 180.0) * v_y) / R_WHEEL;
  }
  //four-wheel kinematics
  else
  {
    //TODO:
    /*
     wheel_vels[0] = sin (60.0 * PI / 180.0) * v_x - cos (60.0 * PI / 180.0) * v_y;
     wheel_vels[1] = sin (0.0 * PI / 180.0) * v_x + cos (0.0 * PI / 180.0) * v_y;
     wheel_vels[2] = -sin (60.0 * PI / 180.0) * v_x - cos (60.0 * PI / 180.0) * v_y;
     wheel_vels[3] = -sin (60.0 * PI / 180.0) * v_x - cos (60.0 * PI / 180.0) * v_y;
     */
  }

  wheels_[0]->setPosition (sign (wheel_vels[0]) * MAXINT);
  wheels_[1]->setPosition (sign (wheel_vels[1]) * MAXINT);
  wheels_[2]->setPosition (sign (wheel_vels[2]) * MAXINT);

  wheels_[0]->setVelocity (abs (wheel_vels[0]));
  wheels_[1]->setVelocity (abs (wheel_vels[1]));
  wheels_[2]->setVelocity (abs (wheel_vels[2]));

}

void
SSLRobot::kick (float speed)
{
  kicker_->setPosition(MAXINT);
  kicker_->setVelocity(speed);

  //TODO:
  // Do sth to check if kick operation is accomplished,
  // and take the kicker back to its initial position.
  kicker_->setPosition(0);
}
void
SSLRobot::dribble (float speed, bool fwd_dir)
{
  dribbler_->setPosition (sign (speed*(2*fwd_dir-1)) * MAXINT);
  dribbler_->setVelocity(speed);
}

int
main ()
{
  std::cout << "hello world!" << std::endl;
  SSLRobot* ssl_robot = new SSLRobot ();
  ssl_robot->run ();

  return 0;
}
