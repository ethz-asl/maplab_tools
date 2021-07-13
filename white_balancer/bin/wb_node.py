#! /usr/bin/env python3

import rospy
from multiprocessing import Lock
import numpy as np
import cv2
'''
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
'''

class WhiteBalancerNode(object):
    def __init__(self):
        print('Foo')
        '''
        self.is_initialized = False

        self.mutex = Lock()
        self.mutex.acquire()
        # Publishers and subscribers.
        in_topic = rospy.get_param("~input_topic")
        out_topic = in_topic + rospy.get_param("~output_topic_suffix")
        rospy.Subscriber(in_topic, Submap, self.img_callback)
        self.out_pub = rospy.Publisher(out_topic, Image, queue_size=10)

        self.is_initialized = True
        self.mutex.release()
        '''



    def img_callback(self, msg):
        if self.is_initialized is False:
            return
        self.mutex.acquire()
        rospy.loginfo(f"[WhiteBalancer] Received img message.")

        self.mutex.release()


if __name__ == '__main__':
    print('bar')
    '''
    rospy.init_node('white_balancer')
    node = WhiteBalancerNode()
    node.spin()
    '''
