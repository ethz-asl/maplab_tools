#! /usr/bin/env python2

import rospy
import time
import numpy as np
from os.path import exists

from datasource import Datasource

class Profiler(object):
    def __init__(self, config, commander):
        self.config = config
        self.commander = commander
        self.ds = Datasource(self.config)

    def start_profiling(self):
        rospy.loginfo('[Profiler] Starting profiling with profile {profile}.'.format(profile=self.config.init_profile))
        if not self.commander.set_profile(self.config.init_profile):
            return

        rospy.loginfo('[Profiler] Starting publishing submaps to the server.')
        if not self.ds.start_publishing_submaps():
            return

        rospy.loginfo('[Profiler] Waiting {secs}s for server completion'.format(secs=self.config.profiling_completion_sleep_time_s))
        self.wait_for_completion()

        rospy.loginfo('[Profiler] Checking the results.')

    def wait_for_completion(self):
        time.sleep(self.config.profiling_completion_sleep_time_s)

    # def check_result(self):
