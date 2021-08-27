#! /usr/bin/env python1

import rospy
import yaml
from os.path import exists
from std_srvs.srv import Empty
from std_msgs.msg import String
from maplab_msgs.srv import DeleteAllRobotMissions, DeleteAllRobotMissionsRequest

class CommandPost(object):
    def __init__(self, config):
        self.is_initialized = False
        self.config = config

        # Publishers and subscribers.
        if config.mode == 'commander':
            self.default_profile = rospy.Service('~maplab_default_profile', Empty, self.default_profile_callback)
            self.high_performance_profile = rospy.Service('~maplab_high_performance_profile', Empty, self.high_performance_profile_callback)
            self.low_performance_profile = rospy.Service('~maplab_low_performance_profile', Empty, self.low_performance_profile_callback)

        self.maplab_reinit_service = rospy.ServiceProxy(self.config.reinit_service_topic, Empty)
        self.maplab_reset_global_map_service = rospy.ServiceProxy(self.config.reset_global_map, DeleteAllRobotMissions)
        self.maplab_whitelist_service = rospy.ServiceProxy(self.config.whitelist_all_missions, Empty)
        self.is_initialized = True

    def default_profile_callback(self, req):
        if self.is_initialized is False:
            return False
        rospy.logwarn('[CommandPost] Called default profile service')
        return self.set_profile(self.config.init_profile)

    def high_performance_profile_callback(self, req):
        if self.is_initialized is False:
            return False
        rospy.logwarn('[CommandPost] Called high performance profile service')
        return self.set_profile(self.config.high_performance_profile)

    def low_performance_profile_callback(self, req):
        if self.is_initialized is False:
            return False
        rospy.logwarn('[CommandPost] Called low performance profile service')
        return self.set_profile(self.config.low_performance_profile)

    def set_profile(self, profile):
        if profile == '':
            return False
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
            rospy.logerr('[CommandPost] Unable to load profile {profile} from {path}. Error: {error}'.format(profile=profile, path=profile_path, error=str(e)))
            return False

        return self.send_reinit_request()

    def load_and_set_profile(self, profile_path):
        with open(profile_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self.set_params_from_dict(data)

    def set_params_from_dict(self, params_dict):
        for key, value in params_dict.items():
            self.try_set_param(key, value)

    def send_reinit_request(self):
        try:
            res = self.maplab_reinit_service()
            return True
        except Exception as e:
            rospy.logerr('[CommandPost] Reinit service at {topic} is not available. Error: {error}'.format(topic=self.config.reinit_service_topic, error=str(e)))
            return False

    def send_global_map_reset(self):
        try:
            success = True
            for robot in self.config.profiling_robots:
                req = DeleteAllRobotMissionsRequest()
                req.robot_name = String(robot)
                rospy.logwarn('[CommandPost] Sending global map reset for robot {robot}'.format(robot=robot))
                res = self.maplab_reset_global_map_service(req)
                success &= res.success.data
            return success
        except Exception as e:
            rospy.logerr('[CommandPost] Reset service at {topic} is not available. Error: {error}'.format(topic=self.config.reset_global_map, error=str(e)))
            return False

    def send_whitelist_request(self):
        try:
            rospy.logwarn('[CommandPost] Sending whitelist request.')
            res = self.maplab_whitelist_service()
            return True
        except Exception as e:
            rospy.logerr('[CommandPost] Whitelist service at {topic} is not available. Error: {error}'.format(topic=self.config.whitelist_all_missions, error=str(e)))
            return False

    def try_set_param(self, key, value):
        service_topic = self.config.maplab_server_prefix + key
        try:
            rospy.set_param(service_topic, value)
            rospy.logdebug('[CommandPost] Setting parameter {param} with value {value}'.format(param=service_topic, value=value))
        except Exception as e:
            rospy.logerr('[CommandPost] Could not set parameter {param} with value {value}. Error: {error}'.format(param=service_topic, value=value, error=str(e)))
            return
