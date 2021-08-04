#! /usr/bin/env python2

import rospy
import yaml
from os.path import exists
from std_srvs.srv import Empty
from maplab_msgs.srv import DeleteAllRobotMissions

class CommandPost(object):
    def __init__(self, config):
        self.config = config

        # Publishers and subscribers.
        self.default_profile = rospy.Service('~maplab_default_profile', Empty, self.default_profile_callback)
        self.maplab_reinit_service = rospy.ServiceProxy(self.config.reinit_service_topic, Empty)
        self.maplab_reset_global_map_service = rospy.ServiceProxy(self.config.reset_global_map, DeleteAllRobotMissions)

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
            rospy.logerr('[CommandPost] Reinit service at {topic} is not available'.format(topic=self.config.reinit_service_topic))
            rospy.logerr(str(e))
            return False

    def send_global_map_reset(self):
        try:
            success = True
            for robot in self.config.profiling_robots:
                req = DeleteAllRobotMissionsRequest()
                req.robot_name = robot
                rospy.logwarn('[CommandPost] Sending global map reset for robot {robot}'.format(robot=robot))
                res = self.maplab_reset_global_map_service(req)
                success &= res.success
            return success
        except Exception as e:
            rospy.logerr('[CommandPost] Reset service at {topic} is not available'.format(topic=self.config.reset_global_map))
            return False

    def try_set_param(self, key, value):
        service_topic = self.config.maplab_server_prefix + key
        try:
            rospy.set_param(service_topic, value)
            rospy.loginfo('[CommandPost] Setting parameter {param} with value {value}'.format(param=service_topic, value=value))
        except Exception as e:
            rospy.logerr('[CommandPost] Could not set parameter {param} with value {value}'.format(param=service_topic, value=value))
            rospy.logerr(str(e))
            return
