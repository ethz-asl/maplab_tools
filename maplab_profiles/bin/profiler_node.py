#! /usr/bin/env python2

import rospy
import time
import yaml
import numpy as np
from os.path import exists
from std_srvs.srv import Empty


class ProfilerNode(object):
    def __init__(self):
        self.is_initialized = False

        # Publishers and subscribers.
        self.default_profile = rospy.Service('~maplab_default_profile', Empty, self.default_profile_callback)

        self.maplab_server_prefix = rospy.get_param("~maplab_server_prefix")
        self.reinit_service_topic = rospy.get_param("~maplab_server_reinit_sevice")
        self.maplab_reinit_service = rospy.ServiceProxy(self.reinit_service_topic, Empty)

        self.init_profile = rospy.get_param("~init_profile")
        self.config_root = rospy.get_param("~config_root")
        self.profiles = rospy.get_param("~profiles")

        self.is_initialized = True
        rospy.loginfo('[MaplabProfilerNode] Initialized. Defined profiles are: ')
        rospy.loginfo('[MaplabProfilerNode] ' + str(self.profiles))
        self.set_profile(self.init_profile)

    def default_profile_callback(self, req):
        if self.is_initialized is False:
            return
        rospy.loginfo('[MaplabProfilerNode] Called default profile service')
        self.set_profile(self.init_profile)

    def set_profile(self, profile):
        if profile not in self.profiles:
            rospy.logerr('Profile {profile} not found in configured profiles.'.format(profile=profile))
            return

        profile_path = self.config_root + profile + '.yaml'
        if not exists(profile_path):
            rospy.logerr('Profile {profile} does not exist in {path}.'.format(profile=profile, path=self.config_root))
            return

        rospy.loginfo('[MaplabProfilerNode] Setting profile: %s' % profile)
        try:
            self.load_and_set_profile(profile_path)
        except Exception as e:
            rospy.logerr('Unable to load profile {profile} from {path}'.format(profile=profile, path=profile_path))
            rospy.logerr(str(e))
            return

        try:
            res = self.maplab_reinit_service()
        except Exception as e:
            rospy.logerr('Service at {topic} is not available'.format(topic=self.reinit_service_topic))
            return

    def load_and_set_profile(self, profile_path):
        with open(profile_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            for key, value in data.items():
                self.try_set_param(key, value)

    def try_set_param(self, key, value):
        service_topic = self.maplab_server_prefix + key
        try:
            rospy.set_param(service_topic, value)
            rospy.loginfo('Setting parameter {param} with value {value}'.format(param=service_topic, value=value))
        except Exception as e:
            rospy.logerr('Could not set parameter {param} with value {value}'.format(param=service_topic, value=value))
            return


if __name__ == '__main__':
    rospy.init_node('profiler_node')
    node = ProfilerNode()
    rospy.spin()
