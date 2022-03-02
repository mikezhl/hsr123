import numpy as np
import math
import sys
import rospy
import hsrb_interface
import pyexotica as exo
import threading
import pickle

from pickup_ik import pickup_ik
from pickup_rrt import pickup_rrt_loop
from pickup_aico import pickup_aico
from image.detect import detect_all
from my_functions import *
import my_arm_client as arm
import my_base_client as base

rospy.init_node("testtest")

robot = hsrb_interface.Robot()
omni_base = robot.get("omni_base")

print(123)