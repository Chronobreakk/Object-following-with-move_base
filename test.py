#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

class RelativeGoalPublisher:
    def __init__(self):
        rospy.init_node('relative_goal_publisher')

        # 订阅AMCL的位置
        self.amcl_pose = None
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        # 订阅MoveBaseSimpleGoal话题
        self.goal_pose = None
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_callback)

        # 发布相对位置的话题
        self.relative_goal_pub = rospy.Publisher('/relative_goal', PoseStamped, queue_size=10)

        self.tf_listener = tf.TransformListener()

    def amcl_pose_callback(self, data):
        self.amcl_pose = data

    def goal_pose_callback(self, data):
        self.goal_pose = data

    def publish_relative_goal(self):
        if self.amcl_pose and self.goal_pose:
            try:
                # 获取AMCL在base_link坐标系下的位置
                (trans, rot) = self.tf_listener.lookupTransform('/base_link', self.amcl_pose.header.frame_id, rospy.Time(0))

                # 获取AMCL的姿态
                euler_angles = euler_from_quaternion(rot)

                # 计算目标位置在base_link坐标系下的相对位置
                relative_goal = PoseStamped()
                relative_goal.header.frame_id = '/base_link'
                relative_goal.header.stamp = rospy.Time.now()
                relative_goal.pose.position.x = self.goal_pose.pose.position.x - trans[0]
                relative_goal.pose.position.y = self.goal_pose.pose.position.y - trans[1]
                relative_goal.pose.position.z = self.goal_pose.pose.position.z - trans[2]

                # 旋转目标姿态到base_link坐标系下
                (roll, pitch, yaw) = euler_angles
                (relative_goal.pose.orientation.x,
                 relative_goal.pose.orientation.y,
                 relative_goal.pose.orientation.z,
                 relative_goal.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)

                # 发布相对位置
                self.relative_goal_pub.publish(relative_goal)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to transform goal pose to base_link")

def main():
    relative_goal_publisher = RelativeGoalPublisher()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        relative_goal_publisher.publish_relative_goal()
        rate.sleep()

if __name__ == '__main__':
    main()
