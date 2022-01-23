#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
import sys
import speechd

from speak_out_loud.msg import SpeakGoal, SpeakAction, SpeakFeedback, SpeakResult, Priority
from std_msgs.msg import String, Bool

class SOLServer(object) :
    """
    Simple node to send text with priority
    """

    def __init__(self):
        self.load_params()
        self.do_i_say_pub = rospy.Publisher("/sol/do_i_say", Bool, queue_size=1)

        self._client = speechd.SSIPClient('sol_ssip_client')
        self._client.set_output_module('rhvoice')
        self._client.set_synthesis_voice(self.defaut_voice)
        self.default_priority = Priority.TEXT
        self.STATUS_UNDEFINED = -1

        rospy.Subscriber("/sol/texts", SpeakGoal, self.speak_task_cb)

        self.server_sol = actionlib.ActionServer('sol_server', SpeakAction, 
                goal_cb=self.speak_action_srv_task_cb, auto_start=False)

        self.server_sol.start()
        rospy.loginfo('SOL server is started')

    def load_params(self):
        self.defaut_voice = rospy.get_param("~default_voice", "elena")

    def speak_action_srv_task_cb(self, req):
        result_msg = SpeakResult()
        goal = req.get_goal()

        if len(goal.text) < 1:
            err_msg = 'there is no text to speak out'
            rospy.logerr(err_msg)
            req.set_rejected(err_msg)
            return

        priority = self.fix_priority(goal.priority)
        msg_id = self.srv_spd_say(priority, req, goal)
        result_msg.msg_id = msg_id
        req.set_accepted()

    def fix_priority(self, priority):
        if not Priority.MIN < priority < Priority.MAX:
            rospy.logwarn("Unknown priority for speaking text. Using default priority = %d", self.default_priority)
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

    def speak_task_cb(self, msg):
        priority = self.fix_priority(msg.priority)
        msg_id = self.spd_say(priority, msg)

    def srv_spd_say(self, priority, req, goal):
        msg_id = -1
        result_msg = SpeakResult()
        self._client.set_synthesis_voice(self.defaut_voice)
        self._client.set_priority(priority)
        if goal.voice=='':
            self._client.set_synthesis_voice(self.defaut_voice)
        else:
            self._client.set_synthesis_voice(goal.voice)
        def callback(callback_type):
            result_msg = SpeakResult()
            if callback_type == speechd.CallbackType.BEGIN:
                self.do_i_say_pub.publish(Bool(True))
            elif callback_type == speechd.CallbackType.END:
                result_msg.msg_id = int(msg_id)
                req.set_succeeded(result_msg)
                self.do_i_say_pub.publish(Bool(False))
            elif callback_type == speechd.CallbackType.CANCEL:
                result_msg.msg_id = int(msg_id)
                req.set_aborted(result_msg)
        spd_result = self._client.speak(goal.text, callback=callback,
                           event_types=(speechd.CallbackType.BEGIN,
                                        speechd.CallbackType.CANCEL,
                                        speechd.CallbackType.END))
        msg_id = int(spd_result[2][0])
        return msg_id

    def spd_say(self, priority, msg):
        msg_id = -1
        self._client.set_synthesis_voice(self.defaut_voice)
        self._client.set_priority(priority)
        if msg.voice=='':
            self._client.set_synthesis_voice(self.defaut_voice)
        else:
            self._client.set_synthesis_voice(msg.voice)
        def callback(callback_type):
            if callback_type == speechd.CallbackType.BEGIN:
                self.do_i_say_pub.publish(Bool(True))
            elif callback_type == speechd.CallbackType.END:
                self.do_i_say_pub.publish(Bool(False))
            elif callback_type == speechd.CallbackType.CANCEL:
                pass
        spd_result = self._client.speak(msg.text, callback=callback,
                           event_types=(speechd.CallbackType.BEGIN,
                                        speechd.CallbackType.CANCEL,
                                        speechd.CallbackType.END))
        msg_id = int(spd_result[2][0])
        return msg_id

    def shutdown(self):
        rospy.logwarn("SOL is closed")
        self._client.close()

if __name__ == '__main__':
    try:
        rospy.init_node('sol_srv')
        rospy.logwarn("Press Ctrl+C to shutdown node")
        solserver = SOLServer()

    except rospy.ROSInterruptException:
        pass

