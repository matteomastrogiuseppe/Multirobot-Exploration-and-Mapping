#!/usr/bin/env python3
import rospy 
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyfmm

from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path 
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from read_msgs.msg import Traj

from path_planner import *
from utils import *


class Robot:
    def __init__(self, name):
        self.name = name

        self.odometry       = Odometry()
        self.twist          = Twist()
        self.prev_command   = Twist()
        self.pose           = Pose2D()
        self.path_pub       = Path()
        self.path_pub.header.frame_id=self.name+"/odom"

        self.goal           = np.array([])
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

        
        self.max_vel = 0.3
        self.safe_dist = 0.5
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
            grid_img = (255 - map > 40).astype(int).astype(float)

        p1 = pose_to_pixel(self.pose, map_info)
        p2 = pose_to_pixel(self.goal.pose, map_info)

        grid_img = self.avoid_other_robots(other_poses, grid_img, map_info)

        p1 = stay_in_grid(p1, grid_img)
        p2 = stay_in_grid(p2, grid_img)

        # Potential Map and Saturation
        GRIDCOSTMAP = np.clip(pyfmm.march(grid_img == 1, batch_size=10000)[0], a_min=0, a_max=12.5)
        #plt.clf()
        #plt.imshow(GRIDCOSTMAP, interpolation='None')
        #plt.colorbar()
        #plt.savefig("costmap.png")
        path = A_STAR(grid_img, GRIDCOSTMAP, p1, p2, k=self.k_obstacles)

        if path is None:
            return True

        trajectory=[]
        self.path_pub.poses=[]
        for point in path:
            x,y = pixel_to_pose(point, map_info)

            posuccia = PoseStamped()            
            posuccia.pose.orientation.w = 1
            posuccia.pose.position.x = y
            posuccia.pose.position.y = x
            posuccia.pose.position.z = 0

            trajectory.append([y, x])
            self.path_pub.poses.append(posuccia)
            #grid_img[point] = 1

        self.path = np.ravel(trajectory).astype(np.float32)
        #cv2.imwrite("with_path.png", grid_img*255)
        return False        
    
    def avoid_other_robots(self, poses, grid_img, map_info):
        """Return the gridmap but with other robots as obstacles."""
        if type(poses) is not list: poses = [poses]
        for pose in poses:
            x,y = pose_to_pixel(pose, map_info)
            grid_img[x-5:x+5,y-5:y+5] = 1

        return grid_img
        


        