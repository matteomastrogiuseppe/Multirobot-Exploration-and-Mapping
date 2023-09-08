#!/usr/bin/env python3
import rospy as rospy
import cv2 

from nav_msgs.msg           import OccupancyGrid
from nav_msgs.msg           import Odometry
from geometry_msgs.msg      import Pose2D
from geometry_msgs.msg      import Pose
from geometry_msgs.msg      import Point
from read_msgs.msg          import PointArray
from visualization_msgs.msg import Marker

from time                   import time, sleep
from copy                   import copy

from get_frontier           import getfrontier
from frontier_finder        import FrontierFinder
from robot                  import Robot
from utils                  import *


class TaskManager:
    def __init__(self):
        self.finder = FrontierFinder()
        print("--- Initialized Finder ---")
        self.k_i = 1
        self.k_c = 2
        self.k_yaw = self.k_c*2

        robot_namelist = rospy.get_param('~robot_namelist', "robot1").split(',')
        self.robot_list = []
        for i in range(len(robot_namelist)):
            self.robot_list.append(Robot(robot_namelist[i]))
        print("--- Created Robots ---")

        

    def spin(self):
        self.finder.spin()
        print("Il finder ha spinnato")
        if len(self.finder.map.shape) != 2 and len(self.finder.map.shape) != 3: 
            print("Assigner did not receive new map. Skipping loop iteration.")
            return 0 
        if len(self.finder.frontiers.frontiers) > 0:
            self.assigner(self.compute_rewards())

        for robot in self.robot_list:
            robot.spin(copy(self.finder.map), self.finder.grid_map.info)
    
    def compute_rewards(self):
        #for i, f in enumerate(self.finder.frontiers.frontiers):
        #    print("Point #", i, "x: ",      f.pose.x)
        #    print("Point #", i, "y: ",      f.pose.y)
        #    print("Point #", i, "theta: ",  f.pose.theta)
        #    print("Point #", i, "length: ", f.length)    
        #    print("\n")

        M = np.zeros((len(self.robot_list), len(self.finder.frontiers.frontiers)))
        for i, robot in enumerate(self.robot_list):
            for j, frontier in enumerate(self.finder.frontiers.frontiers):
            
                rx, ry = robot.odometry.pose.pose.position.x, robot.odometry.pose.pose.position.y 
                r_yaw = get_yaw(robot.odometry)
                
                fx, fy = frontier.pose.x, frontier.pose.y
                f_yaw = frontier.pose.theta

                info_gain = self.k_i*frontier.length
                expl_cost = self.k_c*( np.sqrt((rx-fx)**2+(ry-fy)**2) + self.k_yaw*(r_yaw - (f_yaw+np.pi/2))  )
                M[i,j] = info_gain - expl_cost 
        return M
    
    def assigner(self, M):
        #print(M)
        for i, robot in enumerate(self.robot_list):
            robot.goal = self.finder.frontiers.frontiers[np.argmax(M[i,:])]
            print("Robot #", i ," ha ricevuto il target. x scelta è :", robot.goal.pose.x, ", y è: ", robot.goal.pose.y, ". L'idx è :", np.argmax(M[i,:]))
            


if __name__ == "__main__":
    rospy.init_node('assigner', anonymous=True)
    assigner = TaskManager()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        start = time()
        assigner.spin()
        print("Main loop: ", time()-start, "s")
        rate.sleep()