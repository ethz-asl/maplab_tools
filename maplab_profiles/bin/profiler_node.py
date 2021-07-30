#! /usr/bin/env python2

import rospy
import time
import numpy as np
from std_srvs.srv import Empty


class ProfilerNode(object):
    def __init__(self):
        self.is_initialized = False

        # Publishers and subscribers.
        self.default_profile = rospy.Service('~maplab_default_profile', Empty, self.default_profile_callback)
        init_profile = rospy.get_param("~init_profile")

        self.is_initialized = True
        rospy.loginfo('[MaplabProfilerNode] Initialized.')

    def default_profile_callback(self, req):
        if self.is_initialized is False:
            return
        rospy.loginfo('[MaplabProfilerNode] Called default profile service')

if __name__ == '__main__':
    rospy.init_node('profiler_node')
    node = ProfilerNode()
    rospy.spin()
