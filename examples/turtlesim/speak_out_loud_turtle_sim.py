#!/usr/bin/env python
# coding: utf-8

import rospy

from speak_out_loud.msg import SpeakGoal, Priority
from geometry_msgs.msg import Twist

class TurtleVoice(object) :
    """
    Simple node to send text with priority
    """

    def __init__(self):
        rospy.logwarn("turtleSay node")
        self.goal = SpeakGoal()
        self.goal.sender_node = rospy.get_name()
        self.default_priority = Priority.TEXT
        self.goal.priority = self.default_priority
        self.load_params()

        self.phrases = {'left':'Поворачиваю налево','right':'Поворачиваю right','forward':'move прямо','backward':'move back', 'none':'unknown action'}
        rospy.Subscriber("/turtle1/cmd_vel", Twist, self.speak_out_cb)
        self._pub = rospy.Publisher("/speak_out_loud_texts", SpeakGoal, queue_size=1)
        
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def load_params(self):
        pass

    def speak_out_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        
        if v > 0:
            text_to_say = self.phrases['forward']
        elif v < 0:
            text_to_say = self.phrases['backward']
        elif w < 0:
            text_to_say = self.phrases['right']
        elif w > 0:
            text_to_say = self.phrases['left']
        else:
            text_to_say = self.phrases['none']

        self.goal.text = text_to_say
        self._pub.publish(self.goal)

    def shutdown(self):
        rospy.logwarn("turtleVoice node is closed")

if __name__ == '__main__':
    try:
        rospy.init_node('turtleVoice')
        rospy.logwarn("Press Ctrl+C to shutdown node")
        turtleVoice = TurtleVoice()

    except rospy.ROSInterruptException:
        pass

