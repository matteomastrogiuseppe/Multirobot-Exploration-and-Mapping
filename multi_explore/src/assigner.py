#!/usr/bin/env python3
import rospy as rospy
import rosnode

from multi_explore.msg      import Frontier
from time                   import time

from frontier_finder        import FrontierFinder
from robot                  import Robot
from utils                  import *


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

    def spin(self):        
        # Find new frontiers
        self.finder.spin()

        if len(self.finder.map.shape) != 2 and len(self.finder.map.shape) != 3: 
            print("Assigner did not receive a correct map. Skipping loop iteration.")
            return 0 
        
        # Assign frontiers to robots
        self.assigner()

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

                #if ((robot.pose.x-fx)**2 + (robot.pose.y-fy)**2)**0.5 < 0.3: 
                #    M[i,j] = np.nan
        return M
    
    def assigner(self):
        M = self.compute_rewards()

        # Finished ---> No frontiers to be explored
        finished = False
        if len(self.finder.frontiers.frontiers) == 0:
            print("Finished exploring frontiers, going back to the original position...")
            finished = True

        # Assign frontier to each robot
        for i, robot in enumerate(self.robot_list):  

            # If all frontiers have been explored, go back to original position.
            if finished:
                self.robot_go_home(robot)
                # Shutdown robot when back to original position.
                if abs(robot.pose.x - robot.x0) < 0.1 and \
                   abs(robot.pose.y - robot.y0) < 0.1:
                    self.robot_shutdown(robot)

            else: # Assign best frontier

                try:
                    f_idx = np.nanargmax(M[i,:])
                    robot.goal = self.finder.frontiers.frontiers[f_idx]

                    # Make sure no other robots are assigned to that frontier or close ones:
                    for i, front in enumerate(self.finder.frontiers.frontiers):
                        x,y = front.pose.x, front.pose.y
                        if ((x-robot.goal.pose.x)**2 + (y-robot.goal.pose.y))**0.5 < 2:
                            M[:,i] = np.nan

                except: # All frontiers are already assigned
                    self.robot_go_home(robot)
                
            if robot.change_goal: #To be implemented
                pass
        
        # Shutdown assigner node if all robots are turned off
        if all(robot.shutdown is True for robot in self.robot_list):
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
                

if __name__ == "__main__":
    rospy.init_node('assigner', anonymous=True, disable_signals=True)
    assigner = TaskManager()
    rate = rospy.Rate(rospy.get_param('~assign_rate', default=1))
    
    while not rospy.is_shutdown():
        start = time()
        assigner.spin()
        print("Main loop: ", time()-start, "s")
        rate.sleep()