#! /usr/bin/env python2

import rospy
from multiprocessing import Lock
import numpy as np
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from WBsRGB import WBsRGB
from low_light_enhancement import enhance_image_exposure
from skimage import exposure
from ying import Ying_2017_CAIP
from dhe import dhe
from image_enhancement import image_enhancement
from hdr import *


class WhiteBalancerNode(object):
    def __init__(self):
        self.is_initialized = False

        # Publishers and subscribers.
        in_topic = rospy.get_param("~input_topic")
        out_topic = in_topic + rospy.get_param("~output_topic_suffix")
        rospy.Subscriber(in_topic, Image, self.img_callback)
        self.out_pub = rospy.Publisher(out_topic, Image, queue_size=10)
        self.cv_bridge = CvBridge()
        self.white_balancer = rospy.get_param("~white_balancer")
        self.path_to_models = rospy.get_param("~path_to_models")
        self.debayer_img = rospy.get_param("~debayer_img")
        self.look_up_table = self.compute_lookup_table(gamma=0.85)

        self.is_initialized = True
        rospy.loginfo('[WhiteBalancerNode] Initialized.')
        rospy.loginfo('[WhiteBalancerNode] Listening on topic: ' + in_topic)
        rospy.loginfo('[WhiteBalancerNode] Publishing on topic: ' + out_topic)

    def img_callback(self, msg):
        if self.is_initialized is False:
            return

        try:
            processed_img = self.run_white_balancer(msg)
            self.publish_img(processed_img)
        except Exception as e:
            rospy.logerr('[WhiteBalancerNode] Image processing failed: ' + str(e))

    def run_white_balancer(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        img = self.resize_with_aspect_ratio(img, 416, 416)
        # ie = image_enhancement.IE(img, color_space = 'BGR')
        # img = ie.AGCCPF(1)
        # ie = image_enhancement.IE(img, color_space = 'BGR')
        # img = ie.RSIHE()
        # img = enhance_image_exposure(img, 0.25, 0.1, False,
                                        # sigma=2.5, bc=1, bs=2, be=5, eps=1e-2)
        # img = exposure.adjust_gamma(img, 1.0)
        # img = Ying_2017_CAIP(img, mu=0.2)
        # img = dhe(img)
        # img = exposure.equalize_adapthist(img, clip_limit=0.005, nbins=208)

        # HDR_Filer = FakeHDR(True)
        # img = HDR_Filer.process(img)
        if exposure.is_low_contrast(img, fraction_threshold=0.5):
            rospy.logwarn("Image is bad!!")


        if self.debayer_img:
            img = cv2.cvtColor(img, cv2.COLOR_BAYER_GR2BGR)
            img = cv2.rotate(img, cv2.ROTATE_180)

        if self.white_balancer == 'wb_srgb':
            img = self.run_WB_sRGB(img)
            return cv2.normalize(src=img, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    def run_WB_sRGB(self, img):
        # use upgraded_model= 1 to load our new model that is upgraded with new
        # training examples.
        upgraded_model = 1
        # use gamut_mapping = 1 for scaling, 2 for clipping (our paper's results
        # reported using clipping). If the image is over-saturated, scaling is
        # recommended.
        gamut_mapping = 1
        wbModel = WBsRGB(self.path_to_models, gamut_mapping=gamut_mapping,
                         upgraded=upgraded_model)
        return wbModel.correctImage(img)  # white balance it

    def publish_img(self, img):
        assert self.out_pub != None
        # ie = image_enhancement.IE(img, color_space = 'RGB')
        # img = ie.AGCCPF()

        # ie = image_enhancement.IE(img, color_space = 'RGB')
        # img = ie.RSIHE()
        # img = self.gamma_correction(img)

        # img = enhance_image_exposure(img, 0.25, 0.1, False,
                                        # sigma=2.5, bc=1, bs=2, be=5, eps=1e-2)
        # ie = image_enhancement.IE(img, color_space = 'BGR')
        # img = ie.RSIHE()
        # img = Ying_2017_CAIP(img, mu=0.12)


        image_message = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.out_pub.publish(image_message)

    def compute_lookup_table(self, gamma):
        look_up_table = np.empty((1,256), np.uint8)
        for i in range(256):
            look_up_table[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)

        return look_up_table

    def gamma_correction(self, img):
        return cv2.LUT(img, self.look_up_table)

    def show_image(self, img):
      cv2.imshow('input', img)
      # cv2.imshow('our result', ResizeWithAspectRatio(outImg, width=600)   )
      cv2.waitKey()
      cv2.destroyAllWindows()

    def resize_with_aspect_ratio(self, image, width=None, height=None, inter=cv2.INTER_AREA):
      (h, w) = image.shape[:2]

      if width is None and height is None:
        return image
      if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
      else:
        r = width / float(w)
        dim = (width, int(h * r))

      return cv2.resize(image, dim, interpolation=inter)


if __name__ == '__main__':
    rospy.init_node('white_balancer')
    node = WhiteBalancerNode()
    rospy.spin()
