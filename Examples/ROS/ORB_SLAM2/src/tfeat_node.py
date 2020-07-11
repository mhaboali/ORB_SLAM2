import sys, time

import numpy as np
from scipy.ndimage import filters

import cv2

import roslib
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from ORB_SLAM2.msg import cpp_keypoints

import cv2
from cv_bridge import CvBridge, CvBridgeError


import torchvision as tv
import phototour
import torch
from tqdm import tqdm 
import numpy as np
import torch.nn as nn
import math 
import tfeat_model
import torch.optim as optim
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
import os

import cv2
import numpy as np
import sys
sys.path.append('/home/h/vslam_ws/ORB_SLAM2/Thirdparty/opencv-swig/test/build')
import mat

#init tfeat and load the trained weights
tfeat = tfeat_model.TNet()
models_path = 'pretrained-models'
net_name = 'tfeat-liberty'
tfeat.load_state_dict(torch.load(os.path.join(models_path,net_name+".params")))
# tfeat.cuda()
tfeat.eval()

def run_tfeat(kp1):
    desc_tfeat1 = tfeat_utils.describe_opencv(tfeat, img1, kp1, 32,mag_factor)
   
   desc_tfeat1_mat = mat.Mat.from_array(desc_tfeat1) 

   return desc_tfeat1_mat

def __init__(self):
    rospy.init_node("tfeat_node", anonymous=True)

    # pubs
    # self._head_pub = rospy.Publisher('head_angle', Float64, queue_size=1)
    lift_pub = rospy.Publisher('lift_height', String, queue_size=1)

    # subs
    kp_sub = rospy.Subscriber('keypoints', cpp_keypoints, kp_cb, queue_size=1) 
    loop_rate = 1

    r = rospy.Rate(loop_rate)
    rospy.spin() 