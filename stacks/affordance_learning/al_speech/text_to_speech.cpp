/*
 * main.cpp
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
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
 *     * Neither the name of Kovan Lab nor the names of its
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

#include "iostream"
#include "string"

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "sound_play/SoundRequest.h"

//#include ""
#include "al_acts/TextToSpeechAction.h"
#include "al_utils/al_utils.h"

typedef actionlib::SimpleActionServer<al_acts::TextToSpeechAction> SpeechServer;

class Server
{
protected:
  ros::NodeHandle nh_;
  SpeechServer speech_server_;
  ros::Publisher pub_speech_;

public:
  Server (std::string name) :
    speech_server_ (nh_, name, boost::bind (&Server::executeCallback, this, _1), false)
  {
    pub_speech_ = nh_.advertise<sound_play::SoundRequest> ("/robotsound", 2);

    speech_server_.start ();
  }
  ~Server ()
  {
  }

  void
  executeCallback (const al_acts::TextToSpeechGoalConstPtr &goal_speech)
  {
    bool success = true;
    //assign a speech time considering the length of the string
    std::string speech_text = goal_speech->text;
    std::cout << "sentence: " << speech_text << std::endl;
    std::vector<std::string> words = al::speech::tokenizeSentence (speech_text);
    std::cout << "n_words: " << words.size () << std::endl;
    for (uint i = 0; i < words.size (); i++)
      std::cout << words[i] << std::endl;
    //150-160wpm is a normal speech speed, considering its being a robot, say 120wpm (=2wps)
    ros::Duration time_required (words.size () / 2.0);
    ros::Time t_goal_speech_rcvd = ros::Time::now ();
    bool msg_sent = false;
    ros::Rate r (10);
    while (nh_.ok () && (ros::Time::now () - t_goal_speech_rcvd) < time_required)
    {
      if (!msg_sent)
      {
        msg_sent = true;
        std::cout << "saying " << speech_text << std::endl;

        sound_play::SoundRequest msg_speech;
        msg_speech.sound = sound_play::SoundRequest::SAY;
        msg_speech.command = sound_play::SoundRequest::PLAY_ONCE;
        msg_speech.arg = speech_text.c_str ();
        if (pub_speech_.getNumSubscribers ())
        {
          std::cout << "speech text sent" << std::endl;
          pub_speech_.publish (msg_speech);
        }
      }
      ros::spinOnce ();
      r.sleep ();
    }
    if (!success)
      speech_server_.setAborted ();
    else
      speech_server_.setSucceeded ();
  }
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "al_speech");
  Server server ("speech_action");
  ros::spin ();
  return 0;
}
