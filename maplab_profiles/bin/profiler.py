#! /usr/bin/env python2

import rospy
import time
import numpy as np
import sys
import yaml
import os
from tqdm.auto import tqdm
from os.path import exists
from config import ProfilerConfig
from datasource import Datasource
from command_post import CommandPost
from pose_trajectory_evaluation import PoseTrajectoryEvaluation

class Profiler(object):
    def __init__(self, config, commander):
        self.config = config
        self.commander = commander
        self.ds = Datasource(self.config)

    def start_profiling(self):
        tune_config = {}
        tune_config = self.read_grid_search_params(tune_config)
        tune_config = self.read_choice_params(tune_config)
        if tune_config is None or len(tune_config) == 0:
            rospy.logerr('Config is not correct. Aborting profiling.')
            return

        # Perform profiling
        combinations = self.generate_combinations(tune_config)
        n_combinations = combinations.shape[0]
        rospy.loginfo('[Profiler] Starting profiling with {n_config} config combinations.'.format(n_config=n_combinations))
        losses = []
        est_trajectories = []
        gt_trajectories = []
        for i in range(0, n_combinations):
            tic = time.time()
            configuration = self.create_config_from_combination(tune_config, combinations[i,:])
            est_traj, gt_traj, loss = self.profiling_function(configuration, i, n_combinations)
            if loss > 0.0:
                rospy.loginfo('[Profiler] Loss is {err}.'.format(err=loss))
                losses.append(loss)
                est_trajectories.append(est_traj)
                gt_trajectories.append(gt_traj)

            delta = time.time() - tic
            rospy.loginfo('[Profiler] Profiling took {timing:.2f} seconds'.format(timing=delta))

        if len(losses) == 0:
            rospy.logerr('[Profiler] No profiling results found.')
            return
        losses = np.array(losses)

        # Evaluate results
        n = min(n_combinations, self.config.profiling_show_top_n_configs)
        rospy.loginfo('[Profiler] Getting results for the top {n} config combinations.'.format(n=n))
        top_n_indices = self.get_top_n_configs(losses, n, combinations)
        self.print_results(tune_config, losses, combinations, top_n_indices)
        self.export_results(tune_config, est_trajectories, gt_trajectories, losses, combinations, top_n_indices)


    def print_results(self, tune_config, losses, combinations, top_n_indices):
        i = 1
        for top_n_idx in top_n_indices:
            top_n_combination = combinations[top_n_idx]
            rospy.loginfo('Top {i} config {top_n} is (loss={loss}): '.format(i=i, top_n=top_n_combination, loss=losses[top_n_idx]))
            self.print_combination(tune_config, top_n_combination)
            rospy.loginfo('-----------------------------------------------------------------------------------------')
            i += 1

    def export_results(self, tune_config, est_trajectories, gt_trajectories, losses, combinations, top_n_indices):
        i = 1
        for top_n_idx in top_n_indices:
            top_n_combination = combinations[top_n_idx]

            config_filename = self.config.profiling_export_path + 'config-top-' + str(i) + '.txt'
            self.write_config_to_file(tune_config, top_n_combination, config_filename)

            est_traj_filename = self.config.profiling_export_path + 'est-traj-top-' + str(i) + '.npy'
            gt_traj_filename = self.config.profiling_export_path + 'gt-traj-top-' + str(i) + '.npy'
            np.save(est_traj_filename, est_trajectories[top_n_idx])
            np.save(gt_traj_filename, gt_trajectories[top_n_idx])
            i += 1
        rospy.loginfo('[Profiler] Wrote results to {export_path}'.format(export_path=self.config.profiling_export_path))

    def write_config_to_file(self, tune_config, combination, filename):
        configuration = self.create_config_from_combination(tune_config, combination)
        fo = open(filename, "w")
        for k, v in configuration.items():
            fo.write(str(k) + ': '+ str(v) + '\n')
        fo.close()

    def find_best_config(self, losses, combinations):
        min_loss = np.amin(losses)
        cur_min_idx =  np.where(losses == min_loss)[0]
        return combinations[cur_min_idx, :][0]

    def get_top_n_configs(self, losses, n, combinations):
        best_n = []
        if losses.shape[0] == 0:
            return np.array(best_n)

        sorted_losses = np.sort(losses)
        for i in range(0,n):
            cur_min_idx =  np.where(losses == sorted_losses[i])[0]
            best_n.append(cur_min_idx[0])

        return np.array(best_n)

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
            rospy.loginfo('{key}: {value}'.format(key=key_list[i], value=value_list[i][combination[i]]))

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

    def profiling_function(self, configuration, idx, n_combinations):
        rospy.loginfo('=== Start Profiling {i}/{n} ====================================='.format(i=idx, n=n_combinations))
        rospy.loginfo('[Profiler] Starting profiling with profile {profile}.'.format(profile=self.config.init_profile))
        if not self.commander.set_profile(self.config.init_profile):
            return None, None, -1.0
        self.commander.set_params_from_dict(configuration)
        self.commander.send_reinit_request()

        rospy.loginfo('[Profiler] Starting publishing submaps to the server.')
        if not self.ds.start_publishing_submaps():
            return None, None, -1.0

        rospy.loginfo('[Profiler] Waiting {secs}s for server completion.'.format(secs=self.config.profiling_burnout_sleep_time_s))
        self.wait_for_burnout()

        rospy.loginfo('[Profiler] Checking the results.')
        try:
            est_traj, gt_traj, loss = self.compute_loss_with_gt()
        except:
            rospy.loginfo('[Profiler] Loss computation failed.')
            return None, None, -1.0

        self.commander.send_global_map_reset()
        rospy.loginfo('[Profiler] Waiting {secs}s for server cleanup.'.format(secs=self.config.profiling_completion_sleep_time_s))
        self.wait_for_completion()
        self.commander.send_whitelist_request()
        rospy.loginfo('=== End Profiling ===============================================')
        return est_traj, gt_traj, loss

    def compute_loss_optimized(self):
        pose_filename = 'vertex_poses_velocities_biases.csv'
        est_traj_file = self.config.profiling_merged_map_path + pose_filename
        gt_traj_file = self.config.profiling_ground_truth_path + pose_filename
        if not exists(est_traj_file):
            return sys.maxint
        if not exists(gt_traj_file):
            return sys.maxint
        eval = PoseTrajectoryEvaluation(est_traj_file, gt_traj_file)
        est_traj, gt_traj = eval.compute_synchronized_trajectories()
        return est_traj, gt_traj, eval.compute_trans_ape_rmse(est_traj, gt_traj)

    def compute_loss_with_gt(self):
        pose_filename = 'vertex_poses_velocities_biases.csv'
        est_traj_file = self.config.profiling_merged_map_path + pose_filename
        gt_traj_file = self.config.profiling_ground_truth_file
        if not exists(est_traj_file):
            return sys.maxint
        if not exists(gt_traj_file):
            return sys.maxint
        eval = PoseTrajectoryEvaluation(est_traj_file)
        gt_traj = np.load(gt_traj_file)
        est_traj, gt_traj = eval.compute_synchronized_trajectories_with_evo2(gt_traj)
        return est_traj, gt_traj, eval.compute_evo_trans_ape_rmse(est_traj, gt_traj)

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
