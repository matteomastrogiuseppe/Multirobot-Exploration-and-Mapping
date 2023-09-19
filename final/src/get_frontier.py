#!/usr/bin/env python
from copy import copy

import numpy as np
import cv2
import os

from utils import * 

directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(directory)

def getfrontier(img, Xstart ,Ystart,resolution):
	
	# Reference frames (ROS and cv2) have opposite y-direction. "0" is the vertical direction
	img = cv2.flip(img,0)
	cv2.imwrite('map.jpg', img)
	o=cv2.inRange(img,0,1)
	#cv2.imwrite('o.jpg', o)
	edges = cv2.Canny(img,0,255)
	#cv2.imwrite('edges.jpg', edges)
	contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(o, contours, -1, (255,255,255), 8)
	o=cv2.bitwise_not(o) 
	#cv2.imwrite('o2.jpg', o)
	res = cv2.bitwise_and(o,edges)
	#cv2.imwrite('res.jpg', res)
	#------------------------------

	frontier=copy(res)
	contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frontier, contours, -1, (255,255,255), 2)
	#cv2.imwrite('Contours.jpg', frontier)
	contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	all_pts=[]
	c_list =[]
	if len(contours)>0:
		for i in range(0,len(contours)):
				cnt = contours[i]
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				xr=cx*resolution+Xstart
				yr=cy*resolution+Ystart				
				theta = 0.5*np.arctan2(2*M["mu11"],M["mu20"]-M["mu02"])
				pt=[np.array([xr,-yr,theta, cv2.arcLength(cnt,False)*resolution])]

				# Reference frames (ROS and cv2) have opposite y-direction 
				pt=[np.array([xr,-yr,theta, cv2.arcLength(cnt,False)*resolution])]
				
				#img = cv2.circle(img, (cx,cy), radius=3, color=(0, 0, 255), thickness=-1)
				if len(all_pts)>0:
					all_pts=np.vstack([all_pts,pt])
				else:
					all_pts=pt

	return all_pts
