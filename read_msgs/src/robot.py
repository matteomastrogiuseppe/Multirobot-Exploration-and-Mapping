#!/usr/bin/env python3
import rospy as rospy
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from read_msgs.msg import PointArray
from visualization_msgs.msg import Marker

import matplotlib.pyplot as plt

from time import time, sleep
from path_planner import *
from utils import *


class Robot:
    def __init__(self, name):
        self.name = name

        self.odometry = Odometry()
        self.twist = Twist()
        self.pose = Pose2D()

        self.goal = np.array([])
        self.traj = []
        self.traj_mark = init_marker([1.,0.,0.])

        self.sub = rospy.Subscriber('odom', Odometry, self.callback_odom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_traj = rospy.Publisher('/traject', Marker, queue_size=10)

        self.k_obstacles = 5

    def callback_odom(self,msg):
        self.odometry = msg
        self.pose = Pose2D()
        self.pose.x = self.odometry.pose.pose.position.x
        self.pose.y = self.odometry.pose.pose.position.y
        self.pose.theta = get_yaw(self.odometry)

    def spin(self, map, res):
        self.sub
        self.get_trajectory(map, res)
        #self.pure_pursuit()
        #self.pub.publish(self.twist)
        #print(self.traj_mark.points)
        self.pub_traj.publish(self.traj_mark)

    def get_trajectory(self, map, map_info):
        if len(map.shape)==3:
            map= map.squeeze(-1)
    
        cv2.imwrite("input_map.png", map)
        
        grid_img = (255 - map > 40).astype(int).astype(float)

        self.goal.pose.x, self.goal.pose.y = rot_trasl_2D(p=[self.goal.pose.x, self.goal.pose.y], theta = 0, t = [0,0.0])
        p1 = pose_to_pixel(self.pose, map_info)
        p2 = pose_to_pixel(self.goal.pose, map_info)

        p1 = stay_in_grid(p1, grid_img)
        p2 = stay_in_grid(p2, grid_img)
        print(p1, p2)
        start = time()
        GRIDCOSTMAP = np.clip(pyfmm.march(grid_img == 1, batch_size=10000)[0], a_min=0, a_max=20) #[xvis[0]:xvis[1], yvis[0]:yvis[1]]
        print("pyfmm: ",time()-start)


        start = time()
        path = A_STAR(grid_img, GRIDCOSTMAP, p1, p2, k=self.k_obstacles)
        print("A*:", time()-start)


        grid_img[p1[0], p1[1]] = 10
        grid_img[p2[0], p2[1]] = 10
        cv2.imwrite("init_goal.png", (grid_img*25.5).astype(int))

        start = time()
        self.traj_mark = init_marker([1,0,0], 0.1)
        for point in path:
            p = Point()
            x,y = pixel_to_pose(point, map_info)
            p.x = y
            p.y = x
            self.traj_mark.points.append(p)

            self.traj.append([x, y])

            grid_img[point[0], point[1]] = 5
        
        
        cv2.imwrite("with_path.png", grid_img*255)

    
    def pure_pursuit(self):
        # split trajectory coordinates (local frame)
        xt = list(x[0] for x in self.traj)
        yt = list(x[1] for x in self.traj)

        d_arc = 0
        step = 0
        # move about look ahead distance
        while d_arc < self.look_ahead and step <= len(xt):
            d_arc += np.sqrt((xt[step+1] - xt[step])**2 + (yt[step+1] - yt[step])**2)
            step += 1

        # obtain radius: all coordinates are already in local frame
        L_sq = (xt[step])**2 + (yt[step])**2
        radius = L_sq / (2 * yt[step])

        # yaw = 0 in local frame
        self.error = 1/radius
        # apply PID control to angular vel
        self.cumulative_error = self.error * self.sample_time
        self.vel.angular.z =  self.Kp * self.error + \
                             self.Kd * (self.error - self.prev_error)/self.sample_time +\
                             self.Ki * (self.cumulative_error)
        self.prev_error = self.error
        


        