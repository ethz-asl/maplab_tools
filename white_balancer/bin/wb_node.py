#! /usr/bin/env python2

import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from WBsRGB import WBsRGB
from skimage import exposure
from methods import *

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
        self.resize_img = rospy.get_param("~resize_img")
        self.perform_input_CLAHE = rospy.get_param("~perform_input_CLAHE")
        self.perform_output_log = rospy.get_param("~perform_output_log")
        self.look_up_table = self.compute_lookup_table(gamma=0.75)
        self.wbModel = WBsRGB(self.path_to_models, gamut_mapping=1, upgraded=1)
        self.trigger_only_when_overexposed = rospy.get_param("~trigger_only_when_overexposed")

        self.is_initialized = True
        rospy.loginfo('[WhiteBalancerNode] Initialized.')
        rospy.loginfo('[WhiteBalancerNode] Listening on topic: ' + in_topic)
        rospy.loginfo('[WhiteBalancerNode] Publishing on topic: ' + out_topic)

    def img_callback(self, msg):
        if self.is_initialized is False:
            return
        self.white_balancer = rospy.get_param("~white_balancer")

        try:
            processed_img = self.run_white_balancer(msg)
            if not processed_img is None:
                self.publish_img(processed_img)
        except Exception as e:
            rospy.logerr('[WhiteBalancerNode] Image processing failed: ' + str(e))

    def get_intensity(self, img):
        b = np.array([1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0])
        intensity = np.tensordot(img, b, 1)
        return np.clip(intensity, 0, 255).astype('uint8')

    def is_overexposed(self, img, threshold=20):
        intensity = self.get_intensity(img)
        p1, p50, p90 = np.percentile(intensity, (1, 50, 90))
        return p90 > 250

    def run_white_balancer(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg)

        if self.trigger_only_when_overexposed and not self.is_overexposed(img, 61):
            self.out_pub.publish(msg)
            return None

        rospy.logwarn("Image is badly overexposed. A marvelous sheep is trying to fix it.")
        if self.resize_img:
            img = self.resize_with_aspect_ratio(img, 416, 416)
        if self.perform_input_CLAHE:
            img = exposure.equalize_adapthist(img, clip_limit=0.006, nbins=150)
        if self.debayer_img:
            img = cv2.cvtColor(img, cv2.COLOR_BAYER_GR2BGR)
            img = cv2.rotate(img, cv2.ROTATE_180)

        start_time = time.time()
        if self.white_balancer == 'wb_srgb':
            img = self.wbModel.correctImage(img)
            img = cv2.normalize(src=img, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        elif self.white_balancer == 'retinex':
            return retinex(img)
        elif self.white_balancer == 'retinex_with_adjust':
            return retinex_with_adjust(img)
        elif self.white_balancer == 'stretch':
            return stretch(img)
        elif self.white_balancer == 'max_white':
            return max_white(img)
        elif self.white_balancer == 'simplest_cb':
            img = simplest_cb2(img, 60)
            img = stretch(img)
            img = gamma_trans(img)
        elif self.white_balancer == 'image_analysis':
            img = color_correction_of_image_analysis(img)
        else:
            rospy.logerr("[WhiteBalancerNode] Unknown method specified: " + self.white_balancer)
        print("--- Took %s seconds ---" % (time.time() - start_time))

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayimg = gray
        GLARE_MIN = np.array([0, 0, 50],np.uint8)
        GLARE_MAX = np.array([0, 0, 225],np.uint8)
        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        #HSV + INPAINT
        result = cv2.inpaint(img, frame_threshed, 0.1, cv2.INPAINT_TELEA)
        # CLAHE
        lab1 = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
        lab_planes1 = cv2.split(lab1)
        clahe1 = cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8))
        lab_planes1[0] = clahe1.apply(lab_planes1[0])
        lab1 = cv2.merge(lab_planes1)
        clahe_bgr1 = cv2.cvtColor(lab1, cv2.COLOR_LAB2BGR)

        return clahe_bgr1

    def clahe(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        lab_planes = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=1.6,tileGridSize=(4,4))
        lab_planes[0] = clahe.apply(lab_planes[0])
        lab = cv2.merge(lab_planes)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def publish_img(self, img):
        assert self.out_pub != None
        if self.perform_output_log:
            img = exposure.adjust_log(img, 0.95)
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.out_pub.publish(msg)

    def compute_lookup_table(self, gamma):
        look_up_table = np.empty((1,256), np.uint8)
        for i in range(256):
            look_up_table[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
        return look_up_table

    def gamma_correction(self, img):
        return cv2.LUT(img, self.look_up_table)

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
