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



def execute(cv_img, mask, put_text=False):
    
    """
    Finds image base and bouding box
    cv_img: input image RGB
    mask: binary image mask
    """

    cv_output = cv_img.copy()
    
    # fiding mask contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
    

    # contours parameters
    area = 0
    cont_out = []

    print('New code')

    # fiding great area in the mask
    for c in contours:
        M = cv2.moments(c)        
        if (M["m00"] > area):
            area = M["m00"]
            cont_out = [c]
    
    contours = cont_out

    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)

    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])


    for i in range(len(contours)):
        # contour parameters
        x = boundRect[i][0]
        y = boundRect[i][1]
        w = boundRect[i][2]
        h = boundRect[i][3]

    
    high_corner_x = x
    high_corner_y = y
    low_corner_x = x+w
    low_corner_y = y+h

    # getting the center point of the base of the rectangle
    base_x = low_corner_x - (int(w/2))
    base_y = low_corner_y 
    base = [base_x,base_y]

    # drawning features
    cv2.rectangle(cv_output,
                (high_corner_x,high_corner_y),
                (low_corner_x,low_corner_y),
                (0,255,0),2)

    return base, cv_output
