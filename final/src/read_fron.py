#!/usr/bin/env python3
import rospy as rospy
import time

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from read_msgs.msg import PointArray
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

rospy.init_node('read_front')

def where_front(msg):
    for i, f in enumerate(msg.frontiers):
        print("Point #", i, "x: ",f.pose.x)
        print("Point #", i, "y: ",f.pose.y)
        print("Point #", i, "theta: ",f.pose.theta)
        delta = compare_angles(f.pose.theta)
        print("Point #", i, "length: ", f.length)    
        print("\n")
        
def callback(msg):
    global_map=msg.data
    #print("Occupancy map is a: ", type(msg.data))
    #print("Width: ",msg.info.width)
    #print("Heigth: ",msg.info.height )
    #print("Resolution: ",msg.info.resolution)
    #print("x: ",msg.info.origin.position.x )
    #print("y: ",msg.info.origin.position.y )
    #print("z: ",msg.info.origin.position.z )
    #print("Unexplored: ", map.count(-1),"/",len(map))
    #print("Occupied: ", sum(i > 0.5 for i in global_map),"/",len(global_map))
    print("Callbacked")

def callback_odom(msg):
    global pose 
    pose = msg.pose.pose.orientation

def compare_angles(theta):
    global pose
    _, _ , yaw = euler_from_quaternion([pose.x, pose.y, pose.z, pose.w])
    return yaw - theta
    





while not rospy.is_shutdown():
    rospy.Subscriber("/frontier_poses", PointArray, where_front)
    rospy.Subscriber("/odom", Odometry, callback_odom)


    time.sleep(2)

    
    