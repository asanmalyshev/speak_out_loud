#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib

# from collections import defaultdict
from speak_out_loud.msg import SpeakAction, SpeakGoal, Priority
from std_msgs.msg import String, Bool

class Speaker(object) :
    """
    A hub to forward texts from different nodes on action server 
    """

    def __init__(self):
        self.default_priority = Priority.TEXT
        self.whitelist_on = False
        self.blacklist_on = False
        rospy.Subscriber("/sol/texts", SpeakGoal, self.speak_out_cb)
        rospy.Subscriber("/sol/texts_debug", SpeakGoal, self.speak_out_debug_cb)
        rospy.Subscriber("/sol/whitelist", String, self.whitelist_upd_cb)
        rospy.Subscriber("/sol/blacklist", String, self.blacklist_upd_cb)
        rospy.Subscriber("/sol/whitelist_on", Bool, self.whitelist_on_cb)
        rospy.Subscriber("/sol/blacklist_on", Bool, self.blacklist_on_cb)
        self.client = actionlib.SimpleActionClient('speak_out_loud', SpeakAction)
        self.client.wait_for_server()
        rospy.loginfo("Voice hub is ready to get texts. Server is working")
        self.goal = SpeakGoal()
        self.load_params()
        rospy.spin()

    def whitelist_upd_cb(self, msg):
        if msg.data not in self.whitelist:
            self.whitelist.append(msg.data) 
            rospy.logwarn("Whitelist is updated: %s", self.whitelist)
        self.whitelist_on = True
         
    def whitelist_on_cb(self, msg):
        if msg.data:
            self.whitelist_on = True
            self.blacklist_on = False
            rospy.logwarn("Whitelist is on")
        else:
            self.whitelist_on = False
            rospy.logwarn("Whitelist is off")
         
    def blacklist_upd_cb(self, msg):
        if msg.data not in self.blacklist:
            self.blacklist.append(msg.data) 
            rospy.logwarn("Blacklist is updated: %s", self.blacklist)
        self.blacklist_on = True
         
    def blacklist_on_cb(self, msg):
        if msg.data:
            self.blacklist_on = True
            self.whitelist_on = False
            rospy.logwarn("Blacklist is on")
        else:
            self.blacklist_on = False
            rospy.logwarn("Blacklist is off")
         
    def load_params(self):
        self.node_name = rospy.get_name()
        self.whitelist = rospy.get_param("~whitelist", [])
        self.blacklist = rospy.get_param("~blacklist", [])
        self.debug_mode = rospy.get_param("~debug", False)

        if len(self.whitelist):
            rospy.logwarn("Topic whitelist: %s", self.whitelist)
            self.whitelist_on = True
        if len(self.blacklist):
            rospy.logwarn("Topic blacklist: %s", self.blacklist)
            self.blacklist_on = True
            if len(self.whitelist):
                rospy.logwarn("As there's whitelist, blacklist won't be used")
        
    def speak_out_debug_cb(self, msg):
        if self.debug_mode: # debug output
            msg.priority = self.fix_priority(msg.priority)
            self.client.send_goal(msg)

    def speak_out_cb(self, msg):
        if not (self.whitelist_on or self.blacklist_on): # if there's no lists
            msg.priority = self.fix_priority(msg.priority)
            self.client.send_goal(msg)

        elif self.whitelist_on: # due to whitelist
            if msg.sender_node in self.whitelist:
                msg.priority = self.fix_priority(msg.priority)
                self.client.send_goal(msg)
            else:
                rospy.loginfo("Node %s is not in whitelist. Text won't be read out loud", msg.sender_node)

        elif self.blacklist_on: # due to blacklist
            if msg.sender_node not in self.blacklist:
                msg.priority = self.fix_priority(msg.priority)
                self.client.send_goal(msg)
            else:
                rospy.loginfo("Node %s is in blacklist. Text won't be read out loud", msg.sender_node)

    def fix_priority(self, priority):
        if not Priority.MIN < priority < Priority.MAX:
            rospy.loginfo("Unknown priority for speaking text. Using default priority = %s", self.default_priority)
            priority = self.default_priority
        return priority

if __name__ == '__main__':
    try:
        rospy.init_node('speaker')
        speaker = Speaker()
    except rospy.ROSInterruptException:
        pass

