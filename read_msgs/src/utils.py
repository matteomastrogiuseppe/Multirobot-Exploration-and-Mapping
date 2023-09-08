import rospy
import cv2
import numba as nb
import numpy as np

from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


def init_marker(rgb=[0,1,0], scale = 0.2):
      marker = Marker()
      marker.header.frame_id = "map"
      marker.header.stamp = rospy.Time.now()
      marker.ns = ""
      marker.id = 0
      marker.action = 0
      marker.type = Marker.POINTS
      marker.scale.x = scale
      marker.scale.y = scale
      marker.scale.z = 0*scale
      marker.color.a = 1.0
      marker.color.r = rgb[0]
      marker.color.g = rgb[1]
      marker.color.b = rgb[0]
      marker.points = []

      return marker

@nb.njit(cache=True, parallel=True)
def create_map_raw(data,h,w):
	img = np.zeros((h, w, 1), np.uint8)
	for i in range(h):
		for j in range(w):
			if data[i*w+j]==100:
				img[i,j]=0
			elif data[i*w+j]==0:
				img[i,j]=255
			elif data[i*w+j]==-1:
				img[i,j]=205
	return img


#def create_obstacle_map(data,h,w):
	
def get_yaw(msg):
      _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
						      msg.pose.pose.orientation.y,
						      msg.pose.pose.orientation.z,
						      msg.pose.pose.orientation.w])
      return yaw

def pose_to_pixel(pose: Pose2D(), map_info):
	width = map_info.width
	height = map_info.height
	x_origin = map_info.origin.position.x
	y_origin = map_info.origin.position.y
	res = map_info.resolution
	px = int((pose.x - x_origin)/res)
	py = int((pose.y - y_origin)/res)
	return np.array([py, px])

def pixel_to_pose(p, map_info):
	x_origin = map_info.origin.position.x
	y_origin = map_info.origin.position.y
	res = map_info.resolution

	xr=p[0]*res+x_origin
	yr=p[1]*res+y_origin		

	return np.array([xr, yr])


def rot_trasl_2D(p: [], theta: float, t: []):
	"""First rotation, then translation."""
	x = p[0]*np.cos(theta) - p[1]*np.sin(theta)
	y = p[0]*np.sin(theta) + p[1]*np.cos(theta)
	return np.array([x + t[0],y + t[1]])

@nb.njit(cache=True)
def stay_in_grid(p, grid):
	if grid[p[0],p[1]]:
		raw = np.where(grid==0)
		n = int(len(raw[0]))
		clean = np.zeros((n,2))
		for i in range(n):
			clean[i,:] = [raw[0][i], raw[1][i]]
		dist = np.zeros(len(clean))
		for i, pc in enumerate(clean):
			dist[i] = abs(p[0]-pc[0])+abs(p[1]-pc[1])
		return (clean[np.argmin(dist)]).astype(np.int64)
	else: return p

	