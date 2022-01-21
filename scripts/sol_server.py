#!/usr/bin/env python
# coding: utf-8

import rospy
# import actionlib
import sys
import speechd

from speak_out_loud.msg import SpeakGoal, SpeakAction, SpeakFeedback, Priority
from std_msgs.msg import String, Bool

class SOLServer(object) :
    """
    Simple node to send text with priority
    """

    def __init__(self):
        self.load_params()
        rospy.Subscriber("/sol/texts", SpeakGoal, self.speak_out_cb)
        # do_i_say_pub = rospy.Publisher("/sol/do_i_say", Bool, queue_size=1)

        self._client = speechd.SSIPClient('sol_ssip_client')
        self._client.set_output_module('rhvoice')
        self._client.set_synthesis_voice(self.defaut_voice)

    def load_params(self):
        self.defaut_voice = rospy.get_param("~default_voice", "elena")
        self.default_priority = rospy.get_param("~default_priority", Priority.TEXT)

    def fix_priority(self, priority):
        if not Priority.MIN < priority < Priority.MAX:
            rospy.loginfo("Unknown priority for speaking text. Using default priority = %s", self.default_priority)
            priority = self.default_priority
        # priority mapping
        if priority == Priority.IMPORTANT:
            priority = speechd.Priority.IMPORTANT
        elif priority == Priority.MESSAGE:
            priority = speechd.Priority.MESSAGE
        elif priority == Priority.TEXT:
            priority = speechd.Priority.TEXT
        elif priority == Priority.NOTIFICATION:
            priority = speechd.Priority.NOTIFICATION
        elif priority == Priority.PROGRESS:
            priority = speechd.Priority.PROGRESS

        return priority

    def speak_out_cb(self, msg):
        priority = self.fix_priority(msg.priority)
        msg_id = self.spd_say(priority, msg.text)
        rospy.logwarn(msg_id)

    def spd_say(self, priority, text):
        msg_id = -1
        self._client.set_synthesis_voice(self.defaut_voice)
        self._client.set_priority(priority)
        def callback(callback_type):
            if callback_type == speechd.CallbackType.BEGIN:
                rospy.loginfo("[%s] b",msg_id)
            elif callback_type == speechd.CallbackType.END:
                rospy.loginfo("[%s] e",msg_id)
            elif callback_type == speechd.CallbackType.CANCEL:
                rospy.loginfo("[%s] c",msg_id)

        spd_result = self._client.speak(text, callback=callback,
                           event_types=(speechd.CallbackType.BEGIN,
                                        speechd.CallbackType.CANCEL,
                                        speechd.CallbackType.END))
        msg_id = spd_result[2][0]
        return msg_id

    # def run(self):
    #     pass

    def shutdown(self):
        rospy.logwarn("SOL is closed")
        self._client.close()

if __name__ == '__main__':
    try:
        rospy.init_node('sol_srv')
        rospy.logwarn("Press Ctrl+C to shutdown node")
        solserver = SOLServer()
        # solserver.run()

    except rospy.ROSInterruptException:
        pass

