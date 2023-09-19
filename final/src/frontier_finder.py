#!/usr/bin/env python3
import rospy as rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from read_msgs.msg import PointArray
from read_msgs.msg import Frontier
from visualization_msgs.msg import Marker
from get_frontier import getfrontier
from time import time, sleep
from copy import copy
from utils import *

class FrontierFinder:
      def __init__(self):
            self.sub          = rospy.Subscriber("total_map", OccupancyGrid, self.callback)
            self.pub_rviz     = rospy.Publisher('/frontier_marker', Marker, queue_size=10)
            self.markers      = init_marker()
            self.frontiers    = PointArray()
            self.grid_map     = OccupancyGrid()
            self.map          = np.array([])

            self.received_map = False

      def spin(self):
           self.sub
           while not self.received_map:
                pass
           self.get_frontier() 
           self.pub_rviz.publish(self.markers)      
      
      def callback(self, msg):
            self.grid_map=msg
            self.received_map=True
            
      def get_frontier(self):
            self.map = create_map_raw(np.asarray(self.grid_map.data), self.grid_map.info.height, self.grid_map.info.width)
            points = getfrontier(copy(self.map), 
                                 Xstart   =self.grid_map.info.origin.position.x,
                                 Ystart   =self.grid_map.info.origin.position.y,
                                 resolution=self.grid_map.info.resolution)
            
            self.markers = init_marker()
            self.frontiers.frontiers = []

            for point in points:
                  p = Point()
                  p.x = point[0]
                  p.y = point[1]
                  self.markers.points.append(p)

                  f = Frontier()
                  f.pose.x = point[0]
                  f.pose.y = point[1]
                  f.pose.theta = point[2]
                  f.length = point[3]
                  self.frontiers.frontiers.append(f)
           
if __name__ == "__main__":
      rospy.init_node('listener', anonymous=True)
      r = rospy.Rate(0.5)
      core = FrontierFinder()

      while not rospy.is_shutdown():

            start = time()
            core.spin()
            print("Main loop:", time()-start, " s")
            sleep(1)
      
