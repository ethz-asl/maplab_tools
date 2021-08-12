#! /usr/bin/env python2

import rospy
from os.path import exists
from nav_msgs.msg import Path


class SaverNode(object):
    def __init__(self):
        self.is_initialized = False
        input_path_topic = self.try_get_param('~input_path_topic', '/icp_odometry/poses')
        out_dir = self.try_get_param('~export_directory', '/tmp/')

        rospy.loginfo('[PathSaverNode] Starting with input topic: {input_path_topic}'.format(input_path_topic=input_path_topic))
        rospy.loginfo('[PathSaverNode] Writing ground truth values to {export_dir}'.format(export_dir=out_dir))

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



if __name__ == '__main__':
    rospy.init_node('saver_node')
    node = SaverNode()
    rospy.spin()
