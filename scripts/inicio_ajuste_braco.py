#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#import rospy, time, sys, cv2
#import numpy as np
#import image_lib_v2 as img
#from geometry_msgs.msg import Pose2D
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError

def execute(goal_centroid):
        
  camera_x = 440
  camera_y = 262


  if goal_centroid.x == camera_x and goal_centroid.y == camera_y:
      print('pose central')
  else: 
      print('esta fora do centro') 

