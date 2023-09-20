#!/usr/bin/env python3
import rospy 
import numpy as np

from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from multi_explore.msg import Traj

from time import time
from utils import *

class RobotController:
    def __init__(self, name):
        self.name = name

        self.odometry = Odometry()
        self.twist = Twist()
        self.pose = Pose2D()
        
        self.goal      = [0.,0.]
        self.prev_goal = [0.,0.]
        self.traj = []
        self.received_path = False

        self.sub_odom = rospy.Subscriber(self.name+'/odom', Odometry, self.callback_odom)
        self.sub_path = rospy.Subscriber(self.name+'/traject_points', numpy_msg(Traj), self.callback_path)
        self.pub_vel  = rospy.Publisher(self.name+'/cmd_vel', Twist, queue_size=10)
        
        # Robot coefficients
        self.max_vel        = 0.2
        self.max_ang_vel    = 1
        self.safe_dist      = 1.1
        self.look_ahead     = 0.7
        self.sample_time    = 0.1

        # PID Controller gains and variables 
        self.error = 0
        self.prev_error = 0
        self.kP = 0.45
        self.kD = 0.3
        self.kI = 1

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
        self.sub_odom
        self.sub_path
        
        # Wait for a path
        while not self.received_path:
            pass
        
        self.pure_pursuit()
        self.pub_vel.publish(self.twist)
    
    def pure_pursuit(self):
        # Trajectory data was raveled before publishing. Go back to [x,y] pairs,
        # but in robot reference frame!
        paired = self.traj.reshape((int(self.traj.shape[0]/2), 2))
        traj = []
        for p in paired[::-1]:
            traj.append(rot_trasl_2D_inv(p, self.pose.theta, [self.pose.x, self.pose.y]).tolist())

        # split trajectory coordinates (local frame)
        x = [p[0] for p in traj]
        y = [p[1] for p in traj]

        # Keep track of the difference from previous and new goal. Slow down if large.
        self.goal = traj[-1]
        goal_shift = ( (self.goal[0]-self.prev_goal[0])**2 + (self.goal[1]-self.prev_goal[1])**2 )**0.5

        # Verify distance with the goal
        dist = ( (x[-1])**2 + (y[-1])**2 )**0.5

        # Follow the path carefully if the goal is close
        if dist < self.safe_dist:
            look_ahead = self.look_ahead/4
        elif dist < self.safe_dist/2:
            look_ahead = self.look_ahead/8
        else: 
            look_ahead = 0.7

        d_arc = 0
        step = 0
        # move about look ahead distance
        while d_arc < look_ahead and step <= len(x)-2:
            d_arc += np.sqrt((x[step+1] - x[step])**2 + (y[step+1] - y[step])**2)
            step += 1
    
        # obtain radius of curvature: all coordinates are already in local frame
        L_sq = (x[step])**2 + (y[step])**2
        radius = L_sq / (2 * y[step])

        # yaw = 0 in local frame
        self.error = 1/radius
        # apply PID control to angular vel
        self.cumulative_error = self.error * self.sample_time
        self.twist.angular.z =  self.kP * self.error + \
                                self.kD * (self.error - self.prev_error)/self.sample_time +\
                                self.kI * (self.cumulative_error)
        
        ## Set linear velocity and adjust angular depending on the situation
        # Goal changed suddently
        if goal_shift > 0.5: 
            #print("Sudden change in goal!")
            self.twist.linear.x = 0.05
            self.twist.angular.z /= 3

        # Start to slow down
        elif dist < self.safe_dist/2:
            #print("Slowing down....")
            self.twist.linear.x = 0.08
            self.twist.angular.z /= 2

        # Goal is very close
        elif  dist < self.safe_dist/4: 
            #print("Goal close!")
            self.twist.linear.x = 0.02
            self.twist.angular.z /= 3
        else:
            self.twist.linear.x = self.max_vel            

        # Slow down if large angular twist
        self.twist.linear.x = self.twist.linear.x / (1 + abs(self.twist.angular.z))**2

        # Velocity Saturation
        self.twist.angular.z = max(-self.max_ang_vel, min(self.max_ang_vel, self.twist.angular.z))

        # Update
        self.prev_error = self.error
        self.prev_goal = self.goal



if __name__ == "__main__":
    rospy.init_node('control', anonymous=True)
    name = rospy.get_param('~robot_name', "robot1")
    controller = RobotController(name)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        start = time()
        controller.spin()
        #print("Controller loop: ", time()-start, "s")
        rate.sleep()
    
