import sys
import cv2
import numpy as np
from matplotlib import pyplot as plt


from wls_filter import wlsFilter
from srs import SRS
from virtual_ev import VIG
from tonemap import *


class FakeHDR():

    def __init__(self, flag):
        self.weighted_fusion = flag
        self.wls = wlsFilter
        self.srs = SRS
        self.vig = VIG
        self.tonemap = tonereproduct

    def process(self, image):

        if image.shape[2] == 4:
            image = image[:,:,0:3]
        S = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)/255.0
        image = 1.0*image/255
        L = 1.0*S

        I = self.wls(S, Lambda=0.6, Alpha=0.9)
        R = np.log(L+1e-22) - np.log(I+1e-22)
        R_ = self.srs(R, L)
        I_K = self.vig(L, 1.0 - L)

        return self.tonemap(image, L, R_, I_K, self.weighted_fusion)
