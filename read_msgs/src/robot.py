#!/usr/bin/env python3
import rospy 
import numpy as np
import cv2
import pyfmm

from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from read_msgs.msg import PointArray
from read_msgs.msg import Path
from visualization_msgs.msg import Marker

import matplotlib.pyplot as plt

from time import time, sleep
from copy import deepcopy
from path_planner import *
from utils import *


class Robot:
    def __init__(self, name):
        self.name = name

        self.odometry = Odometry()
        self.twist = Twist()
        self.prev_command = Twist()
        self.max_vel = 0.2
        self.safe_dist = 0.5
        self.pose = Pose2D()

        self.goal = np.array([])
        self.traj = []
        self.path = np.array([])
        self.traj_mark = init_marker([1.,0.,0.])

        self.sub = rospy.Subscriber('odom', Odometry, self.callback_odom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_traj = rospy.Publisher('/'+self.name+'/traject_points', numpy_msg(Path), queue_size=10)
        self.rviz_traj = rospy.Publisher('/'+self.name+'/traject', Marker, queue_size=10)

        self.prev_goal = np.array([0, 0])
        self.change_goal = False

        self.look_ahead = 0.5
        self.sample_time = 0.25
        self.error = 0
        self.prev_error = 0
        self.k_obstacles = 1
        self.kP = 0.3
        self.kD = 0
        self.kI = 0


    def callback_odom(self,msg):
        self.odometry = msg
        self.pose = Pose2D()
        self.pose.x = self.odometry.pose.pose.position.x
        self.pose.y = self.odometry.pose.pose.position.y
        self.pose.theta = get_yaw(self.odometry)

    def spin(self, map, map_info):
        self.sub
        self.change_goal = self.get_trajectory(map, map_info)

        if not self.change_goal:
            #self.pure_pursuit()
            #self.pub.publish(self.twist)
            self.pub_traj.publish(self.path)
            self.rviz_traj.publish(self.traj_mark)

        self.prev_command = deepcopy(self.twist)
            
        

    def get_trajectory(self, map, map_info):
        if len(map.shape)==3:
            map= map.squeeze(-1)
            grid_img = (255 - map > 40).astype(int).astype(float)

        p1 = pose_to_pixel(self.pose, map_info)
        p2 = pose_to_pixel(self.goal.pose, map_info)

        p1 = stay_in_grid(p1, grid_img)
        p2 = stay_in_grid(p2, grid_img)

        # Potential Map Saturation
        GRIDCOSTMAP = np.clip(pyfmm.march(grid_img == 1, batch_size=10000)[0], a_min=0, a_max=20) #[xvis[0]:xvis[1], yvis[0]:yvis[1]]
        print("iniziato A*")
        path = A_STAR(grid_img, GRIDCOSTMAP, p1, p2, k=self.k_obstacles)

        if path is None:
            return True
        #grid_img[p1[0], p1[1]] = 10
        #grid_img[p2[0], p2[1]] = 10
        #cv2.imwrite("init_goal.png", (grid_img*25.5).astype(int))

        start = time()
        self.traj=[]
        self.traj_mark = init_marker([1,0,0], 0.1)
        for point in path:
            p = Point()
            x,y = pixel_to_pose(point, map_info)
            p.x = y
            p.y = x
            self.traj_mark.points.append(p)

            self.traj.append([y, x])
            grid_img[point] = 1

        traj=[]
        for p in self.traj:
            traj.append(rot_trasl_2D_inv(p, self.pose.theta, [self.pose.x, self.pose.y]))
        self.path = np.ravel(traj[::-1]).astype(np.float32)
        #cv2.imwrite("with_path.png", grid_img*255)
        return False

    
    def pure_pursuit(self):
        # split trajectory coordinates (local frame)
        traj = self.path
        

        # Verify distance with the goal
        dist = ( (traj[-1][0])**2 + (traj[-1][1])**2 )**0.5

        # Follow the path carefully if the goal is close
        if dist < self.safe_dist:
            look_ahead = self.look_ahead/2
        else: 
            look_ahead = self.look_ahead 

        d_arc = 0
        step = 0
        # move about look ahead distance
        while d_arc < look_ahead and step <= len(traj)-2:
            d_arc += np.sqrt((traj[step+1][0] - traj[step][0])**2 + (traj[step+1][1] - traj[step][1])**2)
            step += 1
    
        # obtain radius of curvatur: all coordinates are already in local frame
        L_sq = (traj[step][0])**2 + (traj[step][1])**2
        radius = L_sq / (2 * traj[step][1])

        # yaw = 0 in local frame
        self.error = 1/radius
        # apply PID control to angular vel
        self.cumulative_error = self.error * self.sample_time
        self.twist.angular.z =  self.kP * self.error + \
                                self.kD * (self.error - self.prev_error)/self.sample_time +\
                                self.kI * (self.cumulative_error)
        self.prev_error = self.error

        #if self.check_if_stuck():
        #    print("Stuck mode!")
        #    self.twist.linear.x = -0.1
        #    self.twist.angular.z = 0
        #    self.pub.publish(self.twist)
        #    sleep(1)
        #    return 0

        # Goal is very close
        if  dist < self.safe_dist/2:
            self.twist.linear.x = 0.01
            self.twist.angular.z /= 3
        
        # Start to slow down
        elif dist < self.safe_dist:
            self.twist.linear.x = 0.05
            self.twist.angular.z /= 2

        # Full speed
        else:
            self.twist.linear.x = self.max_vel / (1 + abs(self.twist.angular.z))**2


    def check_if_stuck(self):   
        print(self.odometry.twist.twist.linear.x - self.prev_command.linear.x)     
        if abs(self.odometry.twist.twist.linear.x - self.prev_command.linear.x) > 0.1: #or \
            #abs(self.odometry.twist.twist.angular.z - self.prev_command.angular.z) > 0.6:
            return True
        else:
            return False
        
        


        