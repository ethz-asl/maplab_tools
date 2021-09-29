#! /usr/bin/env python2

import rospy
from transfolder_msgs.msg import RobotSubfoldersArray, RobotSubfolders
import operator
import os

class Datasource(object):
    def __init__(self, config):
        self.config = config


    def start_publishing_submaps(self):
        submaps = self.create_timed_submap_dict(self.config.profiling_submap_folder,
            self.config.profiling_robots, self.config.profiling_mode, self.config.profiling_delays,
            self.config.profiling_rate)
        if not submaps:
            rospy.logerr('[Datasource] Unable to create submap dictionary for profiling.')
            return False

        pub = rospy.Publisher(self.config.profiling_notification_topic,
                          RobotSubfoldersArray,
                          queue_size=1000)

        start_time = rospy.Time.now()
        msg = RobotSubfoldersArray()
        robot_to_msg_dict = {}

        publishing = True
        submap_counter = 0
        while publishing:
            time_elapsed = rospy.Time.now() - start_time
            while submaps:
                if self.config.profiling_send_n_submaps > 0 and submap_counter >= self.config.profiling_send_n_submaps:
                    publishing = False
                    break

                notification_time = float(submaps[0][1][0])
                if time_elapsed.to_sec() > notification_time:
                    name = submaps[0][1][1]
                    if not name in robot_to_msg_dict:
                        robot_msg = RobotSubfolders()
                        robot_msg.robot_name = submaps[0][1][1]
                        robot_to_msg_dict[name] = robot_msg
                    robot_to_msg_dict[name].absolute_subfolder_paths.append(
                        submaps[0][0])
                    array_msg = RobotSubfoldersArray()
                    for name in robot_to_msg_dict:
                        array_msg.robots_with_subfolders.append(
                            robot_to_msg_dict[name])
                    rospy.logdebug('[Datasource] Publishing submap for {name}'.format(name=name))
                    pub.publish(array_msg)
                    submaps.pop(0)
                    submap_counter += 1
                else:
                    break

            if not submaps:
                rospy.logdebug('[Datasource] Done publishing submaps.')
                publishing = False
        return True

    def create_timed_submap_dict(self, submap_folder, robots, mode, delays, rate):
        if not os.path.exists(submap_folder):
            rospy.logerr('[Datasource] Submap folder {submap_folder} does not exist.'.format(submap_folder=submap_folder))
            return None

        subfolders_with_paths = [
            os.path.join(submap_folder, f) for f in os.listdir(submap_folder)
            if os.path.isdir(os.path.join(submap_folder, f))
        ]
        robots_in_submap_folder_with_path = {}
        if not subfolders_with_paths:
            rospy.logerr("[Datasource] No robots found in submap folder")
            return None
        for path in subfolders_with_paths:
            robot_name = os.path.basename(path).split("/")[0]
            if robot_name != "merged_map":
                robots_in_submap_folder_with_path[robot_name] = path

        ordered_robots = []
        if robots:
            for robot in robots:
                if not robots_in_submap_folder_with_path.__contains__(robot):
                    rospy.logerr('[Datasource] Robot {robot} does not exist in submap folder.'.format(robot=robot))
                    return None
                ordered_robots.append(robot)
        else:
            ordered_robots = robots

        ordered_submaps_per_robot = {}
        for robot in ordered_robots:
            robot_mission_paths = [
                os.path.join(robots_in_submap_folder_with_path[robot], f)
                for f in os.listdir(robots_in_submap_folder_with_path[robot])
                if os.path.isdir(
                    os.path.join(robots_in_submap_folder_with_path[robot], f))
            ]
            if not robot_mission_paths:
                rospy.logerr('[Datasource] Robot {robot} does not have any missions.'.format(robot=robot))
                return None
            robot_submap_paths = [
                os.path.join(robot_mission_paths[0], f)
                for f in os.listdir(robot_mission_paths[0])
                if os.path.isdir(os.path.join(robot_mission_paths[0], f))
            ]
            robot_submap_paths_ordered = [""] * len(robot_submap_paths)
            for robot_submap_path in robot_submap_paths:
                robot_submap_paths_ordered[int(
                    robot_submap_path.split("_")[-1])] = robot_submap_path
            ordered_submaps_per_robot[robot] = robot_submap_paths_ordered

        timed_submaps_dict = {}
        submap_time = 0

        if mode == 'sequential':
            for robot in ordered_robots:
                for submap in ordered_submaps_per_robot[robot]:
                    timed_submaps_dict[submap] = (submap_time, robot)
                    submap_time += 1. / float(rate)

        elif mode == 'parallel':
            delays_per_robot = {}
            for count, robot in enumerate(ordered_robots):
                if delays:
                    if len(delays) != len(robots):
                        rospy.logwarn('[Datasource] Number of given delays does not match to amount of robots given.')
                    delays_per_robot[robot] = float(delays[count])
                else:
                    delays_per_robot[robot] = 0
                submap_time = delays_per_robot[robot]
                for submap in ordered_submaps_per_robot[robot]:
                    timed_submaps_dict[submap] = (submap_time, robot)
                    submap_time += 1. / float(rate)

        else:
            rospy.logerr('[Datasource] Invalid mode chosen: {mode}.'.format(mode=mode))
            return None
        timed_submaps = sorted(timed_submaps_dict.items(),
                               key=operator.itemgetter(1))
        return timed_submaps
