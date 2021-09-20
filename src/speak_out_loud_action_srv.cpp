#include <iostream>
#include <string>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <actionlib/server/simple_action_server.h>
#include "speak_out_loud/SpeakAction.h"
#include "speak_out_loud/SpeakResult.h"
#include "std_msgs/Bool.h"

#include "libspeechd.h"

using namespace std;

/* 
 * Actionlib server to sound text sent as a service goal
 */

std_msgs::Bool msg_do_i_say;
ros::Publisher do_i_say_pub;
SPDConnection* conn;
string default_voice;

typedef actionlib::SimpleActionServer<speak_out_loud::SpeakAction> Server;

void end_of_speech(size_t msg_id, size_t client_id, SPDNotificationType type)
{
  msg_do_i_say.data = false;
  do_i_say_pub.publish(msg_do_i_say);
}

void begin_of_speech(size_t msg_id, size_t client_id, SPDNotificationType type)
{
  msg_do_i_say.data = true;
  do_i_say_pub.publish(msg_do_i_say);
}


void execute(const speak_out_loud::SpeakGoalConstPtr& goal, Server* as) 
{
  speak_out_loud::SpeakResult result;
  string str_to_play = goal->text;
  SPDPriority priority = static_cast<SPDPriority>(goal->priority);
  string voice = default_voice;

  if (goal->voice != ""){
    voice = goal->voice ;
  }

  spd_set_synthesis_voice(conn, voice.c_str());
  spd_say(conn, priority, str_to_play.c_str());

  as->setSucceeded(result);
}

void srv_sig_handler(int s){
  cout << "Exit speak out loud server" << endl;
  spd_close(conn);
  // exit(1);
}

void spd_config(){
  const char* client_name = ros::this_node::getName().c_str();
  // SPDConnection* conn = spd_open(client_name, NULL, NULL, SPD_MODE_SINGLE);
  conn = spd_open(client_name, NULL, NULL, SPD_MODE_THREADED);
  const string module = "rhvoice";
  spd_set_output_module(conn, module.c_str());
  spd_set_notification_on(conn, SPD_BEGIN);
  spd_set_notification_on(conn, SPD_END);
  // spd_set_notification_on(conn, SPD_CANCEL);

  conn->callback_end = end_of_speech;
  conn->callback_begin = begin_of_speech;
  // conn->callback_cancel = end_of_speech;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sol_srv");
  ros::NodeHandle n;
  do_i_say_pub = n.advertise<std_msgs::Bool>("do_i_say", 10);

  spd_config();

  // signal (SIGINT,srv_sig_handler);
  // signal (SIGTERM,srv_sig_handler);

  n.param<std::string>("sol_srv/default_voice", default_voice);

  // if (n.hasParam("/default_voice")){
  // if (ros::param::has("default_voice")){
  //   cout << "Yup!" << endl;
  // } else {
  //   cout << "No!" << endl;
  // }

	cout << "Default voice: " +  default_voice << endl;
  Server server(n, "sol_internal_action", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}

