#include <iostream>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include "speak_out_loud/SpeakAction.h"
#include "speak_out_loud/SpeakResult.h"

#include "libspeechd.h"

using namespace std;

/* 
 * Actionlib server to sound text sent as a service goal
 */

typedef actionlib::SimpleActionServer<speak_out_loud::SpeakAction> Server;

void execute(const speak_out_loud::SpeakGoalConstPtr& goal, Server* as, string lang) 
{
  speak_out_loud::SpeakResult result;
  
  string str_to_play = goal->text;
  SPDPriority priority;
  priority = static_cast<SPDPriority>(goal->priority);

  string module = "rhvoice";
  string voice_normal;
  string voice_important;

  if (lang == "en"){
    voice_normal = "slt";
    voice_important = "alan";
  }
  else{
    voice_normal = "elena";
    voice_important = "aleksandr";
  }

	// cout << "Text to say: " +  str_to_play << endl;
  const char* client_name = ros::this_node::getName().c_str();
  SPDConnection* conn = spd_open(client_name, NULL, NULL, SPD_MODE_SINGLE);
  spd_set_output_module(conn, module.c_str());

  if (priority == SPD_IMPORTANT) 
    spd_set_synthesis_voice(conn, voice_important.c_str());
  else
    spd_set_synthesis_voice(conn, voice_normal.c_str());

  spd_say(conn, priority, str_to_play.c_str());
  spd_close(conn);

  as->setSucceeded(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speak_out_srv");
  ros::NodeHandle n;
  string lang;

  n.param<std::string>("speak_out_loud_srv/language", lang, "ru");
	cout << "Language to use: " +  lang << endl;
  Server server(n, "speak_out_loud", boost::bind(&execute, _1, &server, lang), false);
  server.start();
  ros::spin();
  return 0;
}

