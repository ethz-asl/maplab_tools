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
        self.white_balancer = rospy.get_param("~wb_srgb")

        self.is_initialized = True
        rospy.loginfo('[WhiteBalancerNode] Initialized.')


    def img_callback(self, msg):
        if self.is_initialized is False:
            return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr('[WhiteBalancerNode] Conversion to image failed: ' + str(e))

    def run_white_balancer(self, img):
        if self.white_balancer == 'wb_srgb':
            return self.run_WB_sRGB(img)


    def run_WB_sRGB(self, img):
        # use upgraded_model= 1 to load our new model that is upgraded with new
        # training examples.
        upgraded_model = 0
        # use gamut_mapping = 1 for scaling, 2 for clipping (our paper's results
        # reported using clipping). If the image is over-saturated, scaling is
        # recommended.
        gamut_mapping = 2
        imshow = 1  # show input/output image

        return None

if __name__ == '__main__':
    rospy.init_node('white_balancer')
    node = WhiteBalancerNode()
    rospy.spin()
