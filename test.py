#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

def dest_pose_callback(msg):
    rospy.loginfo("Received dest pose:")
    global dest_pose
    dest_pose = msg.pose

def amcl_pose_callback(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    rospy.loginfo("Received AMCL pose:")
    rospy.loginfo("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
    rospy.loginfo("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))

def amcl_pose_subscriber():
    rospy.init_node('amcl_pose_subscriber', anonymous=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/move_base_simple/dest', PoseStamped, dest_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        amcl_pose_subscriber()
    except rospy.ROSInterruptException:
        pass
