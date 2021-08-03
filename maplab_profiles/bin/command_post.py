#! /usr/bin/env python2

import rospy
import yaml
from os.path import exists
from std_srvs.srv import Empty

class CommandPost(object):
    def __init__(self, config):
        self.config = config

        # Publishers and subscribers.
        self.default_profile = rospy.Service('~maplab_default_profile', Empty, self.default_profile_callback)
        self.maplab_reinit_service = rospy.ServiceProxy(self.config.reinit_service_topic, Empty)

    def default_profile_callback(self, req):
        if self.is_initialized is False:
            return
        rospy.loginfo('[CommandPost] Called default profile service')
        self.set_profile(self.config.init_profile)

    def set_profile(self, profile):
        if profile not in self.config.profiles:
            rospy.logerr('[CommandPost] Profile {profile} not found in configured profiles.'.format(profile=profile))
            return False

        profile_path = self.config.config_root + profile + '.yaml'
        if not exists(profile_path):
            rospy.logerr('[CommandPost] Profile {profile} does not exist in {path}.'.format(profile=profile, path=self.config.config_root))
            return False

        rospy.loginfo('[CommandPost] Setting profile: %s' % profile)
        try:
            self.load_and_set_profile(profile_path)
        except Exception as e:
            rospy.logerr('[CommandPost] Unable to load profile {profile} from {path}'.format(profile=profile, path=profile_path))
            rospy.logerr(str(e))
            return False

        try:
            res = self.maplab_reinit_service()
        except Exception as e:
            rospy.logerr('[CommandPost] Service at {topic} is not available'.format(topic=self.config.reinit_service_topic))
            return False
        return True

    def load_and_set_profile(self, profile_path):
        with open(profile_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            for key, value in data.items():
                self.try_set_param(key, value)

    def try_set_param(self, key, value):
        service_topic = self.config.maplab_server_prefix + key
        try:
            rospy.set_param(service_topic, value)
            rospy.loginfo('[CommandPost] Setting parameter {param} with value {value}'.format(param=service_topic, value=value))
        except Exception as e:
            rospy.logerr('[CommandPost] Could not set parameter {param} with value {value}'.format(param=service_topic, value=value))
            return