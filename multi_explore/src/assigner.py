#!/usr/bin/env python3
import rospy as rospy
import rosnode

from multi_explore.msg      import Frontier
from time                   import time

from frontier_finder        import FrontierFinder
from robot                  import Robot
from utils                  import *

import os
import pickle


class TaskManager:

    def __init__(self):
        # Create FrontierFinder instance
        self.finder = FrontierFinder()
        print("--- Initialized Finder ---")

        # Create list of Robot istances
        robot_namelist = rospy.get_param('~robot_namelist', "robot1").split(',')
        self.robot_list = []
        for i in range(len(robot_namelist)):
            r = Robot(robot_namelist[i])
            r.x0 = rospy.get_param('~'+r.name+'_x', default=0.)
            r.y0 = rospy.get_param('~'+r.name+'_y', default=0.)
            self.robot_list.append(r)
        print("--- Created Robots ---")

        # Assigner gains 
        self.k_i        = 0.9
        self.k_c        = - 4
        self.k_change   = - 1

        # Store Data
        self.collect_data = True
        self.t_list = []
        self.clean_list = []

    def spin(self):        
        # Find new frontiers
        self.finder.spin()

        if len(self.finder.map.shape) != 2 and len(self.finder.map.shape) != 3: 
            print("Assigner did not receive a correct map. Skipping loop iteration.")
            return 0 
        
        # Assign frontiers to robots
        self.assigner()
        
        self.t_list.append(rospy.get_rostime().to_sec() - self.finder.start_time)
        self.clean_list.append(self.finder.clean_area)

        # Compute trajectories for each robot
        for i, robot in enumerate(self.robot_list):
            other_poses = [other_robot.pose for j, other_robot in enumerate(self.robot_list) if j!=i]
            robot.spin(self.finder.map, self.finder.grid_map.info, other_poses)

        
    # compute the robot-frontier reward matrix
    def compute_rewards(self):

        M = np.zeros((len(self.robot_list), len(self.finder.frontiers.frontiers)))
        for i, robot in enumerate(self.robot_list):
            for j, frontier in enumerate(self.finder.frontiers.frontiers):
            
                rx, ry = robot.odometry.pose.pose.position.x, robot.odometry.pose.pose.position.y                 
                fx, fy = frontier.pose.x, frontier.pose.y

                # Information gain from visiting that frontier
                info_gain = self.k_i*frontier.length

                # Distance robot-frontier
                expl_cost = self.k_c*(np.sqrt((rx-fx)**2+(ry-fy)**2)) 

                # Promote using the old frontier
                change_cost = self.k_change* ((robot.prev_goal[0]-fx)**2 + (robot.prev_goal[1]-fy)**2)
                
                # Final reward function
                M[i,j] = info_gain + expl_cost + change_cost

                # Avoid
                if ((robot.pose.x-fx)**2 + (robot.pose.y-fy)**2) < 0.25: 
                    M[i,j] = -np.nan
        return M
    
    def assigner(self):
        M = self.compute_rewards()

        # Finished ---> No frontiers to be explored
        finished = False
        if len(self.finder.frontiers.frontiers) == 0:
            print("Finished exploring frontiers, going back to the original position...")
            finished = True

            if self.collect_data:
                self.save_data()
                rospy.signal_shutdown("Need time to collect data.")
        
        # Assign frontier to each robot
        assigned = np.full(len(self.robot_list), False)

        for i in range(len(self.robot_list)):

            # If all frontiers have been explored, go back to original position.
            if finished:
                self.robot_go_home(self.robot_list[i])
                # Shutdown robot when back to original position.
                if abs(self.robot_list[i].pose.x - self.robot_list[i].x0) < 0.1 and \
                   abs(self.robot_list[i].pose.y - self.robot_list[i].y0) < 0.1:
                    self.robot_shutdown(self.robot_list[i])

            # Otherwise, assign best frontier
            try:
                r,f = np.unravel_index(np.nanargmax(M, axis=None), M.shape)
                M[r,:] = np.nan; M[:,f] = np.nan
                assigned[r] = True

                self.robot_list[r].goal = self.finder.frontiers.frontiers[f]

                # Make sure no other robots are assigned to that frontier or close ones:
                for k, front in enumerate(self.finder.frontiers.frontiers):
                        x,y = front.pose.x, front.pose.y
                        if ((x-self.robot_list[r].goal.pose.x)**2 + (y-self.robot_list[r].goal.pose.y)**2) < 2:
                            M[:,k] = np.nan

            except: # All frontiers are already assigned
                for j in range(len(self.robot_list)):
                    if not assigned[j]:
                        print("Robot ", self.robot_list[j].name, " idle, going home.")
                        self.robot_go_home(self.robot_list[j])

        # Shutdown assigner node if all robots are turned off
        if all(robot.shutdown is True for robot in self.robot_list):
            self.save_data()
            rospy.signal_shutdown("All robots are off.")
            
    def robot_shutdown(self,robot):
        robot.shutdown = True
        print("All frontiers have been explored and robot: \""+robot.name+"\" is back to initial position.")
        print("Shutting down robot: \""+robot.name+"\".")
        rosnode.kill_nodes([robot.name+'_controller'])                  
        robot.twist.linear.x = 0
        robot.twist.angular.z = 0
        robot.pub_vel.publish(robot.twist)

    def robot_go_home(self,robot):
        msg = Frontier()
        msg.pose.x = robot.x0
        msg.pose.y = robot.y0
        robot.goal = msg
                
    def save_data(self):
        path = os.path.dirname(os.path.abspath(__file__))+'/sim_data_'

        if len(self.robot_list) > 1:
            path = path+'multi/'
        else:
            path = path+'single/'

        if self.k_i == 0:
            path = path+'old_policy/'
        else:
            path = path+'new_policy/'

        for i in range(1,6):
            folder = path+'sim_'+str(i)
            if os.path.exists(folder):
                continue
            else: 
                os.makedirs(folder)
                with open(folder+'/t.pickle', 'wb') as fp:
                    pickle.dump(self.t_list, fp)
                with open(folder+'/map.pickle', 'wb') as fp:
                    pickle.dump(self.clean_list, fp)
                break


if __name__ == "__main__":
    rospy.init_node('assigner', anonymous=True, disable_signals=True)
    assigner = TaskManager()
    rate = rospy.Rate(rospy.get_param('~assign_rate', default=1))
    
    while not rospy.is_shutdown():
        start = time()
        assigner.spin()
        print("Main loop: ", time()-start, "s")
        rate.sleep()