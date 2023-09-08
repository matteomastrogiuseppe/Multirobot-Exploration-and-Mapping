#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped

rospy.init_node('frontier_topic')
my_sub = rospy.Publisher('/frontier', PointStamped, queue_size=10)
point_msg = PointStamped()
point_msg.point.x =0.1
point_msg.point.y=0.2
point_msg.point.z=0

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    my_sub.publish(point_msg)
    point_msg.point.x+=0.05