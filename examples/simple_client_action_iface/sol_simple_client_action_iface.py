#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
import sys

from speak_out_loud.msg import SpeakAction, SpeakGoal, Priority

class Client(object) :
    """
    Simple node to send text with priority
    """

    def __init__(self):
        rospy.logwarn("Simple client with action iface is started.\nType text, set priority - data will be send on sol server to say")
        self._client_sol = actionlib.SimpleActionClient('/sol/action_iface', SpeakAction)
        self._client_sol.wait_for_server()
        self.load_params()
        self.goal = SpeakGoal()
        self.goal.sender_node = rospy.get_name()
        self.default_priority = Priority.TEXT
        self.goal.priority = self.default_priority
        self.goal.debug = self.debug_mode

        rospy.on_shutdown(self.shutdown)

    def load_params(self):
        self.node_name = rospy.get_name()
        rospy.logwarn("Node name is %s",self.node_name)
        self.debug_mode = rospy.get_param("~debug", False)

    def run(self):
        text_to_say = 'Text to say: '
        text_priority = 'Priority [1-5]: '
        while not rospy.is_shutdown():
            try:
                if sys.version_info >= (3,0): #if python version is 3+
                    self.goal.text = input(text_to_say)
                    p = input(text_priority)
                else:
                    self.goal.text = raw_input(text_to_say)
                    p = raw_input(text_priority)

                if not p.isdigit():
                    rospy.logerr("Priority is not digit. Default value =%s is used", self.default_priority)
                    p = self.default_priority

                self.goal.priority = int(p)

                if not Priority.MIN < self.goal.priority < Priority.MAX or self.goal.priority == None:
                    rospy.logerr("Wrong priority format. Default value =%s is used", self.default_priority)
                    self.goal.priority = self.default_priority
                    
                if self.goal.text.strip() != '':
                    self._client_sol.send_goal(self.goal)
                    self._client_sol.wait_for_result()
                    rospy.loginfo(self._client_sol.get_result())
                else:
                    rospy.logerr("No text provided, message won't be send")

            except:
                rospy.logerr("Can't read goal text or priority. No message will be send")

    def shutdown(self):
        rospy.logwarn("Simple node is closed")

if __name__ == '__main__':
    try:
        rospy.init_node('sol_client_example')
        rospy.logwarn("Press Ctrl+C to shutdown node")
        client = Client()
        client.run()

    except rospy.ROSInterruptException:
        pass

