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


def execute(cv_img, mask, put_text=False, draw_contour=False):
    """
    Finds image centroid and contourn
    cv_img: input image RGB
    mask: binary image mask
    """
    
    cv_output = cv_img.copy()
    
    # fiding mask contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
 
    # contours parameters
    area = 0
    moment = []
    cont_out = []
    centroid = [0,0]


    # fiding great area in the mask
    for c in contours:
        M = cv2.moments(c)        
        if (M["m00"] > area):
            area = M["m00"]
            moment = M
            cont_out = [c]
    
    # computing centroid
    centroid[0] = int(moment["m10"]/moment["m00"])
    centroid[1] = int(moment["m01"]/moment["m00"])

    # drawning image output elements
    cv2.circle(cv_output, (centroid[0], centroid[1]), 4, (255,0,0),-1)
    if draw_contour:
        cv2.drawContours(cv_output, cont_out ,-1,(0,255,0),1)

    if put_text:
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (centroid[0],centroid[1])
        fontScale = 0.5
        fontColor = (255,255,255)
        lineType = 1
        text = '('+str(centroid[0])+', '+str(centroid[1]+10)+')'

        cv2.putText(cv_output,text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)


    return centroid, cv_output
