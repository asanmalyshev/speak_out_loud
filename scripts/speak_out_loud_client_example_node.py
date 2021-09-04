#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
import sys

from speak_out_loud.msg import SpeakGoal, Priority

class Client(object) :
    """
    Simple node to send text with priority
    """

    def __init__(self):
        rospy.logwarn("Simple client is started.\nType text, set priority - data will be send on hub to be read")
        self.goal = SpeakGoal()
        self.goal.sender_node = rospy.get_name()
        self.default_priority = Priority.TEXT
        self.goal.priority = self.default_priority
        self.load_params()
        self._pub = rospy.Publisher("/speak_out_loud/texts", SpeakGoal, queue_size=1)
        self._pub_debug = rospy.Publisher("/speak_out_loud/texts_debug", SpeakGoal, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        # rospy.spin()

    def load_params(self):
        self.node_name = rospy.get_name()
        rospy.logerr("My name is %s",self.node_name)
        self.debug_mode = rospy.get_param("~debug", False)

    def run(self):
        text_to_say = 'Text to say: '
        text_priority = 'Priority [1-5]: '
        while not rospy.is_shutdown():
            try:
                if sys.version_info >= (3,0): #if python version is 3+
                    self.goal.text = input(text_to_say)
                    # self.goal.priority = input(text_priority)
                    p = input(text_priority)
                else:
                    self.goal.text = raw_input(text_to_say)
                    # self.goal.priority = int(raw_input(text_priority))
                    p = raw_input(text_priority)

                if not p.isdigit():
                    rospy.logerr("Priority is not digit. Default value =%s is used", self.default_priority)
                    p = self.default_priority

                self.goal.priority = int(p)

                if not Priority.MIN < self.goal.priority < Priority.MAX or self.goal.priority == None:
                    rospy.logerr("Wrong priority format. Default value =%s is used", self.default_priority)
                    self.goal.priority = self.default_priority
                    
                if self.goal.text.strip() != '':
                    if self.debug_mode:
                        self._pub_debug.publish(self.goal)
                    else:
                        self._pub.publish(self.goal)
                else:
                    rospy.logerr("No text provided, message won't be send")

            except:
                rospy.logerr("Can't read goal text or priority. No message will be send")

    def shutdown(self):
        rospy.logwarn("Simple node is closed")

if __name__ == '__main__':
    try:
        rospy.init_node('speaker')
        rospy.logwarn("Press Ctrl+C to shutdown node")
        client = Client()
        client.run()

    except rospy.ROSInterruptException:
        pass

