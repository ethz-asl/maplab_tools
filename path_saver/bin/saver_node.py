#! /usr/bin/env python2

import rospy
from os.path import exists
from nav_msgs.msg import Path


class SaverNode(object):
    def __init__(self):
        self.is_initialized = False
        input_path_topic = self.try_get_param('~input_path_topic', '/icp_odometry/poses')
        self.export_dir = self.try_get_param('~export_directory', '/tmp/')

        rospy.loginfo('[PathSaverNode] Starting with input topic: {input_path_topic}'.format(input_path_topic=input_path_topic))
        rospy.loginfo('[PathSaverNode] Writing ground truth values to {export_dir}'.format(export_dir=self.export_dir))

        rospy.Subscriber(input_path_topic, Path, self.path_callback)
        self.last_seq = -1

        self.is_initialized = True

    def try_get_param(self, key, default=None):
        rospy.logdebug('[PathSaverNode] try_get_param: {key} with default {default}'.format(key=key, default=default))
        return rospy.get_param(key) if rospy.has_param(key) else default

    def path_callback(self, msg):
        if not self.is_initialized:
            return

        if self.last_seq >= msg.header.seq:
            rospy.logwarn('[PathSaverNode] Received an old sequence {seq1} vs. {seq2}'.format(seq1=self.last_seq, seq2=msg.header.seq))
            return
        rospy.loginfo('received msg with {n_nodes} nodes'.format(n_nodes=len(msg.poses)))

        poses = self.parse_path_msg(msg)
        self.export_poses(poses)
        self.last_seq = msg.header.seq

    def parse_path_msg(self, msg):
        n_poses = len(poses)
        poses = np.zeros((n_poses, 8))
        for i in range(n_poses):
            pose_msg = msg.poses[i]
            ts = self.ros_time_to_ns(pose_msg.header.stamp)
            position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
            orientation = np.array([pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z])
            poses[i, 0] = ts
            poses[i, 1:4] = position
            poses[i, 4:8] = orientation
        return poses

    def ros_time_to_ns(self, time):
        k_s_to_ns = 1e9
        return time.secs * k_s_to_ns + time.nsecs

    def export_poses(self, poses):
        poses_file = self.export_dir + 'poses.npy'
        np.save(poses_file, poses)

if __name__ == '__main__':
    rospy.init_node('saver_node')
    node = SaverNode()
    rospy.spin()
