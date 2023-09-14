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
        self.max_vel = 0.3
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
        self.stop_robot = False

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
        if self.goal is None:
            self.stop_robot = True
        self.change_goal = self.get_trajectory(map, map_info)

        if not self.change_goal:
            self.pub_traj.publish(self.path)
            self.rviz_traj.publish(self.traj_mark)

        self.prev_command = deepcopy(self.twist)
        self.prev_goal = np.array([self.goal.pose.x, self.goal.pose.y])
            

    def get_trajectory(self, map, map_info):
        if len(map.shape)==3:
            map= map.squeeze(-1)
            grid_img = (255 - map > 40).astype(int).astype(float)

        p1 = pose_to_pixel(self.pose, map_info)
        p2 = pose_to_pixel(self.goal.pose, map_info)

        p1 = stay_in_grid(p1, grid_img)
        p2 = stay_in_grid(p2, grid_img)

        # Potential Map and Saturation
        GRIDCOSTMAP = np.clip(pyfmm.march(grid_img == 1, batch_size=10000)[0], a_min=0, a_max=10) #[xvis[0]:xvis[1], yvis[0]:yvis[1]]
        #print("iniziato A*")
        path = A_STAR(grid_img, GRIDCOSTMAP, p1, p2, k=self.k_obstacles)

        if path is None:
            return True

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
            #grid_img[point] = 1

        self.path = np.ravel(self.traj).astype(np.float32)
        #cv2.imwrite("with_path.png", grid_img*255)
        return False        
        


        