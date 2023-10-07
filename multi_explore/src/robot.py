#!/usr/bin/env python3
import rospy 
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyfmm
import skfmm

from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path 
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from multi_explore.msg import Traj
from multi_explore.msg import Frontier

from path_planner import *
from utils import *
from time import time


class Robot:
    def __init__(self, name):
        self.name = name

        self.odometry       = Odometry()
        self.twist          = Twist()
        self.prev_command   = Twist()
        self.pose           = Pose2D()
        self.path_pub       = Path()
        self.path_pub.header.frame_id=self.name+"/odom"

        self.goal           = Frontier()
        self.goal.pose.x    = 0
        self.goal.pose.y    = 0
        self.prev_goal      = np.array([0, 0])
        self.path           = np.array([])

        self.change_goal = False
        self.shutdown = False
        self.x0 = 0
        self.y0 = 0

        self.sub = rospy.Subscriber(self.name+'/odom', Odometry, self.callback_odom)
        self.pub_vel = rospy.Publisher(self.name+'/cmd_vel', Twist, queue_size=10)
        self.pub_traj = rospy.Publisher('/'+self.name+'/traject_points', numpy_msg(Traj), queue_size=10)
        self.rviz_traj = rospy.Publisher('/'+self.name+'/traject', Path, queue_size=10)

        # Obstacle avoidance gain
        self.k_obstacles = 2

    def callback_odom(self,msg):
        self.odometry = msg
        self.pose = Pose2D()
        self.pose.x = self.odometry.pose.pose.position.x
        self.pose.y = self.odometry.pose.pose.position.y
        self.pose.theta = get_yaw(self.odometry)

    def spin(self, map, map_info, other_poses):
        self.sub
        self.change_goal = self.get_trajectory(map, map_info, other_poses)

        if not self.change_goal:
            self.pub_traj.publish(self.path)
            self.rviz_traj.publish(self.path_pub)

        self.prev_goal = np.array([self.goal.pose.x, self.goal.pose.y])
            

    def get_trajectory(self, map, map_info, other_poses):

        if len(map.shape)==3:
            map= map.squeeze(-1)

        # Convert gray-scale img in a binary image (obstacles and unexplored --> 1)
        grid_img = (255 - map > 40).astype(int).astype(float)

        #plt.imshow(grid_img[130:290, 130:290])
        np.save("test",grid_img[130:290, 130:290])

        # Convert /map coordinates to pixel coordinates
        p1 = pose_to_pixel(self.pose, map_info)
        p2 = pose_to_pixel(self.goal.pose, map_info)

        # Adjust gridmap to avoid colliding with other robots
        grid_img = self.avoid_other_robots(other_poses, grid_img, map_info)

        # Make sure starting point and goal are in the grid (they could be on the edge)
        p1 = stay_in_grid(p1, grid_img)
        p2 = stay_in_grid(p2, grid_img)

        # Get costmap with Fast Marching Method. Library needs gridmap with zeros as obstacles
        GRIDCOSTMAP = skfmm.distance(1-grid_img)
        # FMM returns distance from obstacles, 
        # so we use the difference squared with respect to the maximum value to create a potential
        GRIDCOSTMAP = np.square(np.clip(np.max(GRIDCOSTMAP) - GRIDCOSTMAP, a_min=4, a_max = np.inf))

        #plt.clf()
        #plt.imshow(GRIDCOSTMAP[130:290, 130:290])
        #plt.title("Fast Marching Method Costmap")
        #plt.colorbar()
        #plt.axis('off')
        #plt.savefig("/imgcostmap.png")

        # A* path planning, as list of points, in image pixels
        path = A_STAR(grid_img, GRIDCOSTMAP, p1, p2, k=self.k_obstacles)

        if path is None:
            return True

        trajectory=[]
        self.path_pub.poses=[]
        for point in path:
            x,y = pixel_to_pose(point, map_info)

            posuccia = PoseStamped()            
            posuccia.pose.orientation.w = 1
            posuccia.pose.position.x = x
            posuccia.pose.position.y = y
            posuccia.pose.position.z = 0

            # Trajectory in /map coordinates
            trajectory.append([x, y])
            # For RViz visualisation
            self.path_pub.poses.append(posuccia)

        # Ravel the trajectory to easily send it as ROS message
        self.path = np.ravel(trajectory).astype(np.float32)
        return False        
    
    def avoid_other_robots(self, poses, grid_img, map_info):
        """Return the gridmap but with other robots as obstacles."""
        if type(poses) is not list: poses = [poses]
        for pose in poses:
            x,y = pose_to_pixel(pose, map_info)
            grid_img[x-5:x+5,y-5:y+5] = 1

        return grid_img
        


        