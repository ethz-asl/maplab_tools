#! /usr/bin/env python2
import rospy

class BaseConfig(object):
    def try_get_param(self, key, default=None):
        rospy.logdebug('[BaseConfig] try_get_param: {key} with default {default}'.format(key=key, default=default))
        return rospy.get_param(key) if rospy.has_param(key) else default

class ProfilerConfig(BaseConfig):
    def __init__(self):
        # general config
        self.maplab_server_prefix = '/maplab_server/maplab_server_node/'
        self.reinit_service_topic = '/maplab_server/reinit_gflags'

        self.init_profile = 'default'
        self.config_root = ''
        self.profiles = ['default']

    def init_from_config(self):
        # general config
        self.maplab_server_prefix = self.try_get_param("~maplab_server_prefix", self.maplab_server_prefix)
        self.reinit_service_topic = self.try_get_param("~maplab_server_reinit_sevice", self.reinit_service_topic)

        self.init_profile = self.try_get_param("~init_profile", self.init_profile)
        self.config_root = self.try_get_param("~config_root", self.config_root)
        self.profiles = self.try_get_param("~profiles", self.profiles)
