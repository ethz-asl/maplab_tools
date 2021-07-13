#! /usr/bin/env python2

import rospy
from multiprocessing import Lock
import numpy as np
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image

class WhiteBalancerNode(object):
    def __init__(self):
        self.is_initialized = False

        # Publishers and subscribers.
        in_topic = rospy.get_param("~input_topic")
        out_topic = in_topic + rospy.get_param("~output_topic_suffix")
        rospy.Subscriber(in_topic, Image, self.img_callback)
        self.out_pub = rospy.Publisher(out_topic, Image, queue_size=10)
        self.cv_bridge = CvBridge()

        self.is_initialized = True
        rospy.loginfo('[WhiteBalancerNode] Initialized.')


    def img_callback(self, msg):
        if self.is_initialized is False:
            return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr('[WhiteBalancerNode] Conversion to image failed: ' + str(e))



if __name__ == '__main__':
    rospy.init_node('white_balancer')
    node = WhiteBalancerNode()
    rospy.spin()
