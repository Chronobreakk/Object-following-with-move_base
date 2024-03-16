#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_multiply, quaternion_inverse

amcl_pose = None
dest_pose = None

def amcl_pose_callback(msg):
    rospy.loginfo("Received AMCL pose:")
    global amcl_pose
    amcl_pose = msg.pose

def dest_pose_callback(msg):
    rospy.loginfo("Received dest pose:")
    global dest_pose
    dest_pose = msg.pose

def calculate_middle_point():
    global amcl_pose, dest_pose
    if amcl_pose is None or dest_pose is None:
        return None
    
    # 获取/amcl_pose的位置和方向
    amcl_position = [amcl_pose.pose.position.x, amcl_pose.pose.position.y, amcl_pose.pose.position.z]
    amcl_orientation = [amcl_pose.pose.orientation.x, amcl_pose.pose.orientation.y, amcl_pose.pose.orientation.z, amcl_pose.pose.orientation.w]
    
    # 获取/dest的位置
    dest_position = [dest_pose.position.x, dest_pose.position.y, dest_pose.position.z]

    middle_position = [0.3*(dest_position[1] - amcl_position[1])/((dest_position[1] - amcl_position[1])**2+(dest_position[0] - amcl_position[0])**2)**0.5, 
                    0.3*(dest_position[0] - amcl_position[0])/((dest_position[1] - amcl_position[1])**2+(dest_position[0] - amcl_position[0])**2)**0.5, 
                    0]

    # 计算中间点的位置
    #middle_position = [(amcl_position[0] + dest_position[0]) / 2,
    #                   (amcl_position[1] + dest_position[1]) / 2,
    #                   (amcl_position[2] + dest_position[2]) / 2]

    # 中间点的方向设置为与/amcl_pose相同
    middle_orientation = amcl_orientation

    return middle_position, middle_orientation

def move_base_goal_publisher():
    rospy.init_node('move_base_goal_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        middle_point = calculate_middle_point()
        if middle_point is not None:
            middle_position, middle_orientation = middle_point
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = middle_position[0]
            goal.pose.position.y = middle_position[1]
            goal.pose.position.z = middle_position[2]
            goal.pose.orientation.x = middle_orientation[0]
            goal.pose.orientation.y = middle_orientation[1]
            goal.pose.orientation.z = middle_orientation[2]
            goal.pose.orientation.w = middle_orientation[3]
            pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
        rospy.Subscriber('/move_base_simple/dest', PoseStamped, dest_pose_callback)
        move_base_goal_publisher()
    except rospy.ROSInterruptException:
        pass
