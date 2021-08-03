#! /usr/bin/env python2

import rospy
import time
import yaml
import numpy as np
from os.path import exists

class Profiler(object):
    def __init__(self, config, commander):
        self.config = config
        self.commander = commander

    def start_profiling(self):
        self.commander.set_profile(self.config.init_profile)
        
