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

def run_tfeat(kp1,kp2):
    desc_tfeat1 = tfeat_utils.describe_opencv(tfeat, img1, kp1, 32,mag_factor)
    desc_tfeat2 = tfeat_utils.describe_opencv(tfeat, img2, kp2, 32,mag_factor)
   
   desc_tfeat1_mat = mat.Mat.from_array(desc_tfeat1) 
   desc_tfeat2_mat = mat.Mat.from_array(desc_tfeat2) 

   return desc_tfeat1_mat, desc_tfeat1_mat

    
"""     all_good_matches = []
    method_goods=[]        
    orb = cv2.ORB_create(nfeatures=1500)
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)
        
    # Apply ratio test
        one_good_pair=()
        good = []
        for m,n in matches:
            if m.distance < 0.8*n.distance:
                good.append([m])
        print(len(good))
        one_good_pair = one_good_pair + (len(good),)
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,0, flags=2) 
        plt.imshow(img3),plt.show()

        desc_tfeat1 = tfeat_utils.describe_opencv(tfeat, img1, kp1, 32,mag_factor)
        desc_tfeat2 = tfeat_utils.describe_opencv(tfeat, img2, kp2, 32,mag_factor)

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(desc_tfeat1,desc_tfeat2, k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.8*n.distance:
                good.append([m])

        print(len(good))
        one_good_pair = one_good_pair + (len(good),)
        method_goods.append(one_good_pair)
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,0, flags=2)

        # plt.imshow(img3),plt.show()
        print("##########################################################################")
    all_good_matches.append(method_goods)
    print(all_good_matches) """