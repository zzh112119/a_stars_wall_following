#!/usr/bin/env python

from visualization_msgs.msg import Marker
#from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rospy

# We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
publisher = rospy.Publisher('/visualization_wall_following', Marker, queue_size="1000")

marker = Marker()
direction = 'center'

# Input data is Odometry
def callback(data):
# Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
    marker.header.frame_id = "/laser"
    marker.pose.position.x = data.pose.pose.position.x
    marker.pose.position.y = data.pose.pose.position.y
    marker.pose.position.z = data.pose.pose.position.z # or set this to 0

    marker.type = marker.SPHERE

    marker.scale.x = 0.1 # If marker is too small in Rviz can make it bigger here
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    if direction == 'left':
        print('Left - red')
        marker.color.r = 1.0
        marker.color.g = 0.0
    elif direction == 'right':
        print('Right - green')
        marker.color.r = 0.0
        marker.color.g = 1.0
    else:
        print('Center - yellow')
        marker.color.r = 1.0
        marker.color.g = 1.0
    marker.color.b = 0.0

    # Publish the MarkerArray
    print("Sending marker")
    publisher.publish(marker)

def colorCallback(data):
    print(data)
    direction = data.data

if __name__ == '__main__':
    rospy.init_node('visualize_wall_following')
    rospy.Subscriber('/vesc/odom', Odometry, callback)
    rospy.Subscriber('/direction_string', String, colorCallback)
    rospy.spin()
