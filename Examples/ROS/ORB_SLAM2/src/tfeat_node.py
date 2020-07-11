#!/usr/bin/env python

import sys, time

import numpy as np
from scipy.ndimage import filters

import cv2

import roslib
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

sys.path.append('/home/h/vslam_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/lib/python2.7/dist-packages/ORB_SLAM2')
from ORB_SLAM2.msg import cpp_keypoints

from cv_bridge import CvBridge, CvBridgeError


import torchvision as tv
import phototour
import tfeat_utils
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
# sys.path.append('/home/h/vslam_ws/ORB_SLAM2/Thirdparty/opencv-swig/test/build')
# import mat


#init tfeat and load the trained weights
tfeat = tfeat_model.TNet()
models_path = '/home/h/vslam_ws/ORB_SLAM2/Thirdparty/tfeat/pretrained-models'
net_name = 'tfeat-liberty'
tfeat.load_state_dict(torch.load(os.path.join(models_path,net_name+".params")))
tfeat.cuda()
tfeat.eval()

def run_tfeat(kp1):
    desc_tfeat1 = tfeat_utils.describe_opencv(tfeat, img1, kp1, 32,mag_factor)
   
    # desc_tfeat1_mat = mat.Mat.from_array(desc_tfeat1) 

    return desc_tfeat1

def kp_cb(msg):
    pass

if __name__ == '__main__':
    rospy.init_node("tfeat_node", anonymous=True)
    print("##########\nBefore Tfeat \n#############\n")
    # pubs
    # self._head_pub = rospy.Publisher('head_angle', Float64, queue_size=1)
    tfeat_pub = rospy.Publisher('tfeat_descriptors', Float32MultiArray, queue_size=1)
    
    img1 = cv2.imread('/home/h/vslam_ws/ORB_SLAM2/Thirdparty/tfeat/imgs/v_churchill/1.ppm',0) 
    orb = cv2.ORB_create(nfeatures=1500)
    kp1, des1 = orb.detectAndCompute(img1, None)
    mag_factor = 3
    # print(kp1)
    desc_tfeat1 = tfeat_utils.describe_opencv(tfeat, img1, kp1, 32,mag_factor)
    desc_list = np.ndarray.tolist(desc_tfeat1)
    desc_list = desc_list.flatten()
    print(desc_tfeat1.shape)
    desc_msg = Float32MultiArray()
    desc_layout = MultiArrayDimension()
    desc_layout.label = "desc_w"
    desc_layout.size = len(desc_list)
    desc_layout.stride = len(desc_list)
    desc_msg.layout.dim.append(desc_layout)
    # desc_layout.label = "desc_h"
    # desc_layout.size = 128
    # desc_layout.stride = 128
    # desc_msg.layout.dim.append(desc_layout)
    desc_msg.data = desc_list
    tfeat_pub.publish(desc_msg)
    print("##########\nAfter Tfeat \n#############\n")

    # subs
    kp_sub = rospy.Subscriber('keypoints', cpp_keypoints, kp_cb, queue_size=1) 
    loop_rate = 1

    r = rospy.Rate(loop_rate)
    rospy.spin() 