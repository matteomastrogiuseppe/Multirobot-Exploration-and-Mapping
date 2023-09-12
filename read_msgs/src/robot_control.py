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

class RobotController:
    def __init__(self, name):
        self.name = name
        self.traj_topic = '/'+name+'/traject_points'

        self.odometry = Odometry()
        self.twist = Twist()
        self.prev_command = Twist()
        self.max_vel = 0.2
        self.safe_dist = 0.5
        self.pose = Pose2D()

        self.goal = np.array([])
        self.traj = []
        self.received_path = False

        self.sub_odom = rospy.Subscriber('odom', Odometry, self.callback_odom)
        self.sub_path = rospy.Subscriber(self.traj_topic, numpy_msg(Path), self.callback_path)

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.prev_goal = np.array([0, 0])
        self.change_goal = False

        self.look_ahead = 0.5
        self.sample_time = 0.1
        self.error = 0
        self.prev_error = 0
        self.k_obstacles = 1
        self.kP = 0.4
        self.kD = 0.1
        self.kI = 0.1

    def callback_odom(self,msg):
        self.odometry = msg
        self.pose = Pose2D()
        self.pose.x = self.odometry.pose.pose.position.x
        self.pose.y = self.odometry.pose.pose.position.y
        self.pose.theta = get_yaw(self.odometry)
    
    def callback_path(self,msg):
        self.traj = msg.points
        self.received_path = True

    def spin(self):
        while not self.received_path:
                pass

        self.sub_odom
        self.sub_path
        
        self.pure_pursuit()
        self.pub_vel.publish(self.twist)
    
    def pure_pursuit(self):
        # split trajectory coordinates (local frame)
        x = self.traj[0:-2:2]
        y = self.traj[1:-1:2]
        # Verify distance with the goal
        dist = ( (x[-1])**2 + (y[-1])**2 )**0.5
        #print("Dist: ", dist)

        # Follow the path carefully if the goal is close
        if dist < self.safe_dist:
            look_ahead = self.look_ahead/2
        else: 
            look_ahead = self.look_ahead 

        d_arc = 0
        step = 0
        # move about look ahead distance
        while d_arc < look_ahead and step <= len(x)-2:
            d_arc += np.sqrt((x[step+1] - x[step])**2 + (y[step+1] - y[step])**2)
            step += 1
    
        # obtain radius of curvatur: all coordinates are already in local frame
        L_sq = (x[step])**2 + (y[step])**2
        radius = L_sq / (2 * y[step])

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
        #    self.pub_vel.publish(self.twist)
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
            print("Full speed")
            self.twist.linear.x = self.max_vel / (1 + abs(self.twist.angular.z))**2


    def check_if_stuck(self):   
        print(self.odometry.twist.twist.linear.x - self.prev_command.linear.x)     
        if abs(self.odometry.twist.twist.linear.x - self.prev_command.linear.x) > 0.1: #or \
            #abs(self.odometry.twist.twist.angular.z - self.prev_command.angular.z) > 0.6:
            return True
        else:
            return False

if __name__ == "__main__":
    name = rospy.get_param('~robot_name', "robot1")
    rospy.init_node(name+'_control', anonymous=True)
    controller = RobotController(name)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        start = time()
        controller.spin()
        print("Controller loop: ", time()-start, "s")
        rate.sleep()
