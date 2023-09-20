from geometry_msgs.msg import Pose2D

pose = Pose2D()
pose2 = Pose2D()
print(type([pose]))
for pose in [pose]:
    print(pose)
print([pose, pose2])