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
        self.reset_global_map = '/maplab_server/delete_all_robot_missions'

        self.init_profile = 'default'
        self.config_root = ''
        self.profiles = ['default']

        self.profiling_notification_topic = '/maplab_server/map_update_notification'
        self.profiling_submap_folder = ''
        self.profiling_send_n_submaps = -1
        self.profiling_robots = []
        self.profiling_mode = 'parallel'
        self.profiling_delays = []
        self.profiling_rate = 0.1
        self.profiling_burnout_sleep_time_s = 60
        self.profiling_completion_sleep_time_s = 60
        self.profiling_merged_map_path = '/tmp/maplab_server/merged_map/'
        self.profiling_ground_truth_path = ''
        self.profiling_grid_search_params_file = 'grid_search'
        self.profiling_show_top_n_configs = 5

    def init_from_rosparams(self):
        # general config
        self.mode = self.try_get_param("/maplab_profiles/mode", self.mode)
        self.maplab_server_prefix = self.try_get_param("/maplab_profiles/maplab_server_prefix", self.maplab_server_prefix)
        self.reinit_service_topic = self.try_get_param("/maplab_profiles/maplab_server_reinit_sevice", self.reinit_service_topic)
        self.reset_global_map = self.try_get_param("/maplab_profiles/maplab_reset_global_map", self.reset_global_map)

        self.init_profile = self.try_get_param("/maplab_profiles/init_profile", self.init_profile)
        self.config_root = self.try_get_param("/maplab_profiles/config_root", self.config_root)
        self.profiles = self.try_get_param("/maplab_profiles/profiles", self.profiles)

        self.profiling_notification_topic = self.try_get_param("/maplab_profiles/profiling_notification_topic", self.profiling_notification_topic)
        self.profiling_submap_folder = self.try_get_param("/maplab_profiles/profiling_submap_folder", self.profiling_submap_folder)
        self.profiling_send_n_submaps = self.try_get_param("/maplab_profiles/profiling_send_n_submaps", self.profiling_send_n_submaps)
        self.profiling_robots = self.try_get_param("/maplab_profiles/profiling_robots", self.profiling_robots)
        self.profiling_mode = self.try_get_param("/maplab_profiles/profiling_mode", self.profiling_mode)
        self.profiling_delays = self.try_get_param("/maplab_profiles/profiling_delays", self.profiling_delays)
        self.profiling_rate = self.try_get_param("/maplab_profiles/profiling_rate", self.profiling_rate)
        self.profiling_burnout_sleep_time_s = self.try_get_param("/maplab_profiles/profiling_burnout_sleep_time_s", self.profiling_burnout_sleep_time_s)
        self.profiling_completion_sleep_time_s = self.try_get_param("/maplab_profiles/profiling_completion_sleep_time_s", self.profiling_completion_sleep_time_s)
        self.profiling_merged_map_path = self.try_get_param("/maplab_profiles/profiling_merged_map_path", self.profiling_merged_map_path)
        self.profiling_ground_truth_path = self.try_get_param("/maplab_profiles/profiling_ground_truth_path", self.profiling_ground_truth_path)
        self.profiling_grid_search_params_file = self.try_get_param("/maplab_profiles/profiling_grid_search_params_file", self.profiling_grid_search_params_file)
        self.profiling_show_top_n_configs = self.try_get_param("/maplab_profiles/profiling_show_top_n_configs", self.profiling_show_top_n_configs)
