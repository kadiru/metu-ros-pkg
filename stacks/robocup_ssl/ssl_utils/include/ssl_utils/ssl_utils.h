/*
 * ssl_utils.h
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

#ifndef SSL_UTILS_H_
#define SSL_UTILS_H_

#include "ros/ros.h"
#include "string"

namespace ssl
{
  namespace math
  {
    const float PI = 3.14159265;

    float
    degToRad (const float deg);

    float
    radToDeg (const float rad);

    template<class T>
      int
      sign (T data)
      {
        if (data < 0)
          return -1;
        else
          return 1;
      }
  }

  namespace naming
  {
    namespace team
    {
      const std::string BLUE_TEAM = "blue";
      const std::string YELLOW_TEAM = "yellow";

      enum TeamEnum
      {
        BLUE,
        YELLOW
      };
    }

    namespace frame
    {
      const std::string FIELD = "field";
      const std::string BASE_LINK = "base_link";
    }

    namespace topic
    {
      const std::string RAW_GLOBAL_STATE = "raw_global_state";
      const std::string EST_GLOBAL_STATE = "est_global_state";
    }

    inline std::string
    createRobotName (const uint team, const uint id)
    {
      std::string robot_name;
      if (team == team::BLUE)
        robot_name.assign (ssl::naming::team::BLUE_TEAM);
      else
        robot_name.assign (ssl::naming::team::YELLOW_TEAM);
      std::stringstream s;
      s << id;
      robot_name.append (s.str ());
      return robot_name;
    }

    inline std::string
    createRobotName (const std::string team, const uint id)
    {
      std::string robot_name = team;
      std::stringstream s;
      s << id;
      robot_name.append (s.str ());
      return robot_name;
    }

  }

  namespace config
  {
    namespace sim
    {
      const uint TIME_STEP = 16;
    }

    namespace team
    {
      const uint TEAM_CAPACITY = 5;
    }

    namespace game
    {
      const double FIELD_WIDTH = 6.050; //in x
      const double FIELD_HEIGHT = 4.050;//in y
      const double FIELD_OUTER_WIDTH = 6.550;
      const double FIELD_OUTER_HEIGHT = 4.550;
      const double BALL_RADIUS = 0.043;//m
    }
  }

  namespace visual
  {
    enum ROBOT_VISUAL_STATE
    {
      OUT_OF_FOV,//out of field of view
      NO_MOVE, //sth fishy may be going on...
      INSIDE_FIELD, OUTSIDE_FIELD
    };

    enum ROBOT_GAME_STATE
    {
      IN, OUT
    };
  }
}

#endif /* SSL_UTILS_H_ */
