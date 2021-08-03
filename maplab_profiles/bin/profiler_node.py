#! /usr/bin/env python2

import rospy

from os.path import exists
from std_srvs.srv import Empty

from config import ProfilerConfig
from command_post import CommandPost
from profiler import Profiler

class ProfilerNode(object):
    def __init__(self):
        self.is_initialized = False
        self.config = ProfilerConfig()
        self.config.init_from_config()

        # Initialize with the default profile.
        self.commander = CommandPost(self.config)

        if self.config.mode == 'profiler':
            self.profiler = Profiler(self.config, self.commander)
            self.profiler.start_profiling()
        else:
            self.commander.set_profile(self.config.init_profile)
            rospy.loginfo('[MaplabProfilerNode] Initialized. Defined profiles are: ')
            rospy.loginfo('[MaplabProfilerNode] ' + str(self.config.profiles))

        self.is_initialized = True


if __name__ == '__main__':
    rospy.init_node('profiler_node')
    node = ProfilerNode()
    rospy.spin()
