#!/usr/bin/env python


#--------Include modules---------------
from copy import copy

import numpy as np
import cv2
import os
import time
import numba as nb

from utils import * 

directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(directory)

#-----------------------------------------------------


def getfrontier(img, Xstart ,Ystart,resolution):
	
	# Reference frames (ROS and cv2) have opposite y-direction. "0" is the vertical direction
	img = cv2.flip(img,0)
	cv2.imwrite('map.jpg', img)
	#img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
	o=cv2.inRange(img,0,1)
	#cv2.imwrite('o.jpg', o)
	edges = cv2.Canny(img,0,255)
	#cv2.imwrite('edges.jpg', edges)
	contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(o, contours, -1, (255,255,255), 5)
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
				#start = time.time()
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				xr=cx*resolution+Xstart
				yr=cy*resolution+Ystart				
				theta = 0.5*np.arctan2(2*M["mu11"],M["mu20"]-M["mu02"])
				pt=[np.array([xr,-yr,theta, cv2.arcLength(cnt,False)*resolution])]
				#print("Time with contours: ", time.time()-start)

				#start=time.time()
				#ellipse  = cv2.fitEllipse(cnt)
				#(x,y),(MA,ma),angle = ellipse
				#xr2 = x*resolution+Xstart
				#yr2 = y*resolution+Ystart
				#length = max(MA,ma)*resolution
				#pt=[np.array([xr2,-yr2,(angle-np.pi)*np.pi/180, length])]
				#print("Time with ellipses: ", time.time()-start)
				#c_list.append(ellipse)
				#print("Xdiff: ", xr-xr2, "Ydiff: ", yr-yr2, "Thetadiff: ", theta-(angle-np.pi)/180*np.pi, "Major axis: ",max(MA,ma)*resolution)

				# Reference frames (ROS and cv2) have opposite y-direction 
				pt=[np.array([xr,-yr,theta, cv2.arcLength(cnt,False)*resolution])]
				
				#img = cv2.circle(img, (cx,cy), radius=3, color=(0, 0, 255), thickness=-1)
				if len(all_pts)>0:
					all_pts=np.vstack([all_pts,pt])
				else:
					all_pts=pt

	### Check frontiers orientation
	#for i,c in enumerate(c_list):	
	#	cx = c[0]
	#	cy = c[1]	
	#	theta = c[2]			
	#	img2 = cv2.line(img, (int(cx-100*np.cos(theta)),int(cy-100*np.sin(theta))), (int(cx+100*np.cos(theta)),int(cy+100*np.sin(theta))), color=(0, 255, 0), thickness=2) 
	#	theta+= np.pi/2
	#	img2 = cv2.line(img2, (int(cx-10*np.cos(theta)),int(cy-10*np.sin(theta))), (int(cx+10*np.cos(theta)),int(cy+10*np.sin(theta))), color=(0, 125, 125), thickness=1) 
	#	cv2.imwrite('with_points_'+str(i)+'.jpg', img2)
	#	img2=cv2.ellipse(img,c_list[i], (0,0,255), 1)
	#	cv2.imwrite('with_points_'+str(i)+'.jpg', img2)
	#cv2.imwrite('with_points.jpg', img)

	return all_pts
