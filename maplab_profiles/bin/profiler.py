#! /usr/bin/env python2

import rospy
import time
import numpy as np
import sys
import yaml
import os
from os.path import exists
from ray import tune
import ray
from multiprocessing import Lock
from functools import partial
from config import ProfilerConfig
from datasource import Datasource
from command_post import CommandPost
from pose_trajectory_evaluation import PoseTrajectoryEvaluation
from locker import *

class Locker(object):
    instance = None
    mutex = Lock()

    def __new__(cls):
        if cls.instance is None:
            cls.instance = super(Locker, cls).__new__(cls)
            # Put any initialization here.
        return cls.instance

def compute_loss(config):
    pose_filename = 'vertex_poses_velocities_biases.csv'
    est_traj_file = config.profiling_merged_map_path + pose_filename
    gt_traj_file = config.profiling_ground_truth_path + pose_filename
    if not exists(est_traj_file):
        return sys.maxint
    if not exists(gt_traj_file):
        return sys.maxint
    eval = PoseTrajectoryEvaluation(est_traj_file, gt_traj_file)
    return eval.compute_ape()

class Profiler(object):
    def __init__(self, config, commander):
        self.config = config
        self.commander = commander
        self.ds = Datasource(self.config)

    def start_profiling(self):
        tune_config = {}
        tune_config = self.read_grid_search_params(tune_config)
        tune_config = self.read_choice_params(tune_config)

        # Perform profiling
        combinations = self.generate_combinations(tune_config)
        n_combinations = combinations.shape[0]
        rospy.loginfo('[Profiler] Starting profiling with {n_config} config combinations.'.format(n_config=n_combinations))
        losses = []
        for i in range(0, n_combinations):
            configuration = self.create_config_from_combination(tune_config, combinations[i,:])
            loss = self.profiling_function(configuration)
            if loss > 0.0:
                losses.append(loss)
        if len(losses) == 0:
            rospy.logerr('[Profiler] No profiling results found.')
            return
        losses = np.array(losses)

        # Evaluate results
        n = min(n_combinations, self.config.profiling_show_top_n_configs)
        top_n = self.get_top_n_configs(losses, n, combinations)
        print('\nTop {n} configs: '.format(n=n))
        for i in range(0, n):
            print('Top {i} config {top_n} is: '.format(i=i, top_n=top_n[i,:]))
            self.print_combination(tune_config, top_n[i,:])
            print('--------------------')

    def find_best_config(self, losses, combinations):
        min_loss = np.amin(losses)
        cur_min_idx =  np.where(losses == min_loss)[0]
        return combinations[cur_min_idx, :][0]

    def get_top_n_configs(self, losses, n, combinations):
        best_n = []
        if losses.shape[0] == 0:
            return np.array(best_n).squeeze()

        sorted_losses = np.sort(losses)
        for i in range(0,n):
            cur_min_idx =  np.where(losses == sorted_losses[i])[0]
            best_n.append(combinations[cur_min_idx, :])
        return np.array(best_n).squeeze()

    def generate_combinations(self, config):
        values = [np.arange(0,len(x)) for x in config.values()]
        grid = np.meshgrid(*values)
        return np.array(grid).T.reshape(-1,len(values))

    def print_all_combinations(self, combinations):
        n_combinations = combinations.shape[0]
        for i in range(0, n_combinations):
            rospy.loginfo('combination {i} is {combination}'.format(i=i+1, combination=combinations[i,:]))

    def print_combination(self, config, combination):
        n_params = len(combination)
        key_list = list(config.keys())
        value_list = list(config.values())
        for i in range(0, n_params):
            rospy.loginfo('param {key} has value {value}'.format(key=key_list[i], value=value_list[i][combination[i]]))

    def create_config_from_combination(self, config, combination):
        n_params = len(combination)
        key_list = list(config.keys())
        value_list = list(config.values())
        configuration = {}
        for i in range(0, n_params):
            configuration[key_list[i]] = value_list[i][combination[i]]
        return configuration

    def read_grid_search_params(self, tune_config):
        grid_search_params_file = self.config.config_root + self.config.profiling_grid_search_params_file + '.yaml'
        if not exists(grid_search_params_file):
            rospy.logwarn('Found no grid search parameters.')
            return tune_config

        try:
            with open(grid_search_params_file) as f:
                data = yaml.load(f, Loader=yaml.FullLoader)
                for key, value in data.items():
                    tune_config[key] = value
        except Exception as e:
            rospy.logerr('[Profiler] Unable to read and set grid search params at {file}'.format(file=grid_search_params_file))
            rospy.logerr(e)
        return tune_config

    def read_choice_params(self, tune_config):
        return tune_config

    def profiling_function(self, configuration):
        rospy.loginfo('[Profiler] Starting profiling with profile {profile}.'.format(profile=self.config.init_profile))
        if not self.commander.set_profile(self.config.init_profile):
            return -1.0
        self.commander.set_params_from_dict(configuration)
        self.commander.send_reinit_request()

        rospy.loginfo('[Profiler] Starting publishing submaps to the server.')
        if not self.ds.start_publishing_submaps():
            return -1.0

        rospy.loginfo('[Profiler] Waiting {secs}s for server completion'.format(secs=self.config.profiling_burnout_sleep_time_s))
        self.wait_for_burnout()

        rospy.loginfo('[Profiler] Checking the results.')
        loss = self.compute_loss()
        self.commander.send_global_map_reset()
        rospy.loginfo('[Profiler] Waiting {secs}s for server cleanup.'.format(secs=self.config.profiling_completion_sleep_time_s))
        self.wait_for_completion()
        self.commander.send_whitelist_request()
        return loss

    def compute_loss(self):
        pose_filename = 'vertex_poses_velocities_biases.csv'
        est_traj_file = self.config.profiling_merged_map_path + pose_filename
        gt_traj_file = self.config.profiling_ground_truth_path + pose_filename
        if not exists(est_traj_file):
            return sys.maxint
        if not exists(gt_traj_file):
            return sys.maxint
        eval = PoseTrajectoryEvaluation(est_traj_file, gt_traj_file)
        return eval.compute_ape()

    def wait_for_burnout(self):
        time.sleep(self.config.profiling_burnout_sleep_time_s)

    def wait_for_completion(self):
        time.sleep(self.config.profiling_completion_sleep_time_s)

    def check_result(self):
        pose_filename = 'vertex_poses_velocities_biases.csv'
        est_traj_file = self.config.profiling_merged_map_path + pose_filename
        gt_traj_file = self.config.profiling_ground_truth_path + pose_filename
        if not exists(est_traj_file):
            rospy.logerr('[Profiler] Estimated trajectory does not exist at {est_traj_file}'.format(est_traj_file))
            return sys.maxint
        if not exists(gt_traj_file):
            rospy.logerr('[Profiler] Ground truth trajectory does not exist at {opt_traj_file}'.format(opt_traj_file))
            return sys.maxint
        eval = PoseTrajectoryEvaluation(est_traj_file, gt_traj_file)
        return eval.compute_ape()
