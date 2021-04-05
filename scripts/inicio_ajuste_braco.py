#!/usr/bin/env python3
# -*- coding: utf-8 -*-



def execute(goal_centroid):
        
  camera_x = 440
  camera_y = 262


  if goal_centroid.x == camera_x and goal_centroid.y == camera_y:
      print('pose central')
  else: 
      print('esta fora do centro') 

