#! /usr/bin/env python2
import rospy

class BaseConfig(object):
    def try_get_param(self, key, default=None):
        rospy.logdebug('[BaseConfig] try_get_param: {key} with default {default}'.format(key=key, default=default))
        return rospy.get_param(key) if rospy.has_param(key) else default

class ProfilerConfig(BaseConfig):
    def __init__(self):
        # general config
        self.mode = 'commander'
        self.maplab_server_prefix = '/maplab_server/maplab_server_node/'
        self.reinit_service_topic = '/maplab_server/reinit_gflags'

        self.init_profile = 'default'
        self.config_root = ''
        self.profiles = ['default']

        self.profiling_notification_topic = 'transfolder_receiver/folder_received'
        self.profiling_submap_folder = ''
        self.profiling_robots = []
        self.profiling_mode = 'parallel'
        self.profiling_delays = []
        self.profiling_rate = 0.1
        self.profiling_completion_sleep_time_s = 60

    def init_from_config(self):
        # general config
        self.mode = self.try_get_param("~mode", self.mode)
        self.maplab_server_prefix = self.try_get_param("~maplab_server_prefix", self.maplab_server_prefix)
        self.reinit_service_topic = self.try_get_param("~maplab_server_reinit_sevice", self.reinit_service_topic)

        self.init_profile = self.try_get_param("~init_profile", self.init_profile)
        self.config_root = self.try_get_param("~config_root", self.config_root)
        self.profiles = self.try_get_param("~profiles", self.profiles)

        self.profiling_notification_topic = self.try_get_param("~profiling_notification_topic", self.profiling_notification_topic)
        self.profiling_submap_folder = self.try_get_param("~profiling_submap_folder", self.profiling_submap_folder)
        self.profiling_robots = self.try_get_param("~profiling_robots", self.profiling_robots)
        self.profiling_mode = self.try_get_param("~profiling_mode", self.profiling_mode)
        self.profiling_delays = self.try_get_param("~profiling_delays", self.profiling_delays)
        self.profiling_rate = self.try_get_param("~profiling_rate", self.profiling_rate)
        self.profiling_completion_sleep_time_s = self.try_get_param("~profiling_completion_sleep_time_s", self.profiling_completion_sleep_time_s)
