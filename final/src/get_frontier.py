#!/usr/bin/env python
import numpy as np
import cv2
import os

from utils import * 

directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(directory)

def getfrontier(img, Xstart ,Ystart,resolution):
	
	# Reference frames (ROS and cv2) have opposite y-direction. "0" is the vertical direction
	img = cv2.flip(img,0)
	obstacles =cv2.inRange(img,0,1)
	edges = cv2.Canny(img,0,255)
	large_obst, _ = cv2.findContours(obstacles,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(obstacles, large_obst, -1, (255,255,255), 8)
	clean =cv2.bitwise_not(obstacles) 

	frontiers = cv2.bitwise_and(clean,edges)
	contours, _ = cv2.findContours(frontiers,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frontiers, contours, -1, (255,255,255), 2)
	# Further filtering
	contours, _ = cv2.findContours(frontiers,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#cv2.imwrite('Frontiers.jpg', frontiers)

	# Contours to frontiers centroids
	all_pts=[]
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
				
				img = cv2.circle(img, (cx,cy), radius=3, color=(0, 0, 255), thickness=-1)
				if len(all_pts)>0:
					all_pts=np.vstack([all_pts,pt])
				else:
					all_pts=pt

	#cv2.imwrite('with_points.jpg', img)
	return all_pts
