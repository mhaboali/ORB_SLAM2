#!/usr/bin/env python

import sys, time

import numpy as np
from scipy.ndimage import filters

import cv2

import roslib
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from cv_bridge.srv import get_desc
## /home/h/vslam_ws/ORB_SLAM2/Thirdparty/tfeat
sys.path.append('/home/h/vslam_ws/ORB_SLAM2/Thirdparty/tfeat')
## sys.path.append('/home/h/vslam_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/lib/python2.7/dist-packages/ORB_SLAM2')
# from ORB_SLAM2.msg import cpp_keypoints

from cv_bridge import CvBridge, CvBridgeError
import message_filters

print(sys.path)

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

# sys.path.append('/home/h/vslam_ws/ORB_SLAM2/Thirdparty/opencv-swig/test/build')
# import mat


#init tfeat and load the trained weights
tfeat = tfeat_model.TNet()
models_path = '/home/h/vslam_ws/ORB_SLAM2/Thirdparty/tfeat/pretrained-models'
net_name = 'tfeat-liberty'
tfeat.load_state_dict(torch.load(os.path.join(models_path,net_name+".params")))
tfeat.cuda()
tfeat.eval()

bridge = CvBridge()
tfeat_pub = rospy.Publisher('tfeat_descriptors', Float32MultiArray, queue_size=1)


def run_tfeat(kp1):
    desc_tfeat1 = tfeat_utils.describe_opencv(tfeat, img1, kp1, 32,mag_factor)
   
    # desc_tfeat1_mat = mat.Mat.from_array(desc_tfeat1) 

    return desc_tfeat1

def get_tft_desc(req):
    global bridge, tfeat_pub
    input_img = req.img
    input_header = req.header
    img1 = bridge.imgmsg_to_cv2(input_img, "mono8")
    orb = cv2.ORB_create(nfeatures=1500)
    kp1, des1 = orb.detectAndCompute(img1, None)
    mag_factor = 3
    # print(kp1)
    desc_tfeat1 = tfeat_utils.describe_opencv(tfeat, img1, kp1, 32,mag_factor)
    desc_list = np.ndarray.tolist(desc_tfeat1)
    desc_msg = Float32MultiArray()
    desc_layout = MultiArrayDimension()
    desc_msg.layout.dim = []
    dims = np.array(desc_tfeat1.shape)
    
    for i in range(0,dims[0]): #should be rather fast. 
        # gets the num. of dims of nparray to construct the message
        desc_msg.layout.dim.append(MultiArrayDimension())
        desc_msg.layout.dim[i].size = dims[1]
        desc_msg.layout.dim[i].stride = 0
        desc_msg.layout.dim[i].label = 'dim_%d'%i
    
    print("\n#####\nPub-desc\n######") 
    desc_msg.data = np.frombuffer(desc_tfeat1.tobytes(),'float32') ## serializes
    tfeat_pub.publish(desc_msg)

if __name__ == '__main__':
    rospy.init_node("tfeat_node", anonymous=True)
    # imgLeft_sub = rospy.Subscriber('/camera/image_raw', Image, callback)
    get_tfeat_desc = rospy.Service('tfeat/get_desc', get_desc, get_tft_desc)
    
    rospy.spin() 
