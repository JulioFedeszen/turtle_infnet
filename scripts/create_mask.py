#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Library for image processing using        #
# OpensCV.                                  #
#                                           #
# Changes:                                  #
#    * Updated to Python3 and OPencv 4.2.0  #
#    * Get the great area in the image to   #
#      compute centroid and base point.     #
#                                           #
# Author: Adalberto Oliveira                #
# Autonomous Vehicle - Infnet	            #
# Version: 1.22                             #
# Date: 21 mar 2021                         #
#                                           #
#############################################
import rospy, time, sys, cv2
import numpy as np
import image_lib_v2 as img
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def execute(image, low, high, im_blur=False):
    """
    Receives an image and lower and upper values for color segmentation
    image: a RGB type image
    low, high: numpy array
    im_blur: applying Gaussian blur
    """


    # converting from RGB to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # creating mask
    mask = cv2.inRange(hsv,low, high)
    
    # applying Gaussian smoothing    
    if im_blur:
        mask = cv2.GaussianBlur(mask,(15,15),20)

    return mask

