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

def training_function(config):
    # global_locker.mutex.acquire()
    print('---------------------------- start --------------------------------')
    # print(global_locker)
    node_name = 'profiler_function_' + str(os.getpid())
    # rospy.init_node(node_name, disable_signals=True)
    profiler_config = ProfilerConfig()
    profiler_config.init_from_rosparams()

    # commander = CommandPost(profiler_config)
    # ds = Datasource(profiler_config)

    # if not commander.set_profile(profiler_config.init_profile):
        # tune.track.log(mean_loss=100)
    # commander.set_params_from_dict(config)
    # commander.send_reinit_request()

    # if not ds.start_publishing_submaps():
        # tune.track.log(mean_loss=200)

    # time.sleep(10)
    # tune.report(mean_loss = self.check_result()) only for python3
    # tune.track.log(mean_loss=compute_loss(profiler_config))
    tune.track.log(mean_loss=300)
    # commander.send_global_map_reset()
    # time.sleep(60)
    # rospy.signal_shutdown('done profiling.')
    # global_locker.mutex.release()

class Profiler(object):
    def __init__(self, config):
        self.config = config
        # self.commander = commander
        # self.ds = Datasource(self.config)

    def start_profiling(self):
        tune_config = {}
        tune_config = self.read_grid_search_params(tune_config)
        tune_config = self.read_choice_params(tune_config)

        rospy.loginfo('[Profiler] Starting profiling with config:')
        rospy.loginfo(tune_config)
        # analysis = tune.run(training_function, config)
        # train_func = partial(Profiler.profiling_function, self=self)
        analysis = tune.run(
            training_function,
            stop={
                "training_iteration": 1,
            },
            verbose=1,
            config = tune_config,
            resources_per_trial={'cpu': 1},
            num_samples = 1)

        print("Best config: ", analysis.get_best_config(metric="mean_loss", mode="min"))

    def read_grid_search_params(self, tune_config):
        grid_search_params_file = self.config.config_root + self.config.profiling_grid_search_params_file + '.yaml'
        if not exists(grid_search_params_file):
            rospy.logwarn('Found no grid search parameters.')
            return tune_config

        try:
            with open(grid_search_params_file) as f:
                data = yaml.load(f, Loader=yaml.FullLoader)
                for key, value in data.items():
                    tune_config[key] = tune.grid_search(value)
        except Exception as e:
            rospy.logerr('[Profiler] Unable to read and set grid search params at {file}'.format(file=grid_search_params_file))
            rospy.logerr(e)
        return tune_config

    def read_choice_params(self, tune_config):
        return tune_config

    def profiling_function(self, config):
        rospy.loginfo('[Profiler] Starting profiling with profile {profile}.'.format(profile=self.config.init_profile))
        if not self.commander.set_profile(self.config.init_profile):
            tune.report(mean_loss = sys.maxint)
        self.commander.set_params_from_dict(config)
        self.commander.send_reinit_request()

        rospy.loginfo('[Profiler] Starting publishing submaps to the server.')
        if not self.ds.start_publishing_submaps():
            tune.report(mean_loss = sys.maxint)

        rospy.loginfo('[Profiler] Waiting {secs}s for server completion'.format(secs=self.config.profiling_completion_sleep_time_s))
        self.wait_for_completion()

        rospy.loginfo('[Profiler] Checking the results.')
        # tune.report(mean_loss = self.check_result()) only for python3
        tune.track.log(mean_loss=intermediate_score)

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
