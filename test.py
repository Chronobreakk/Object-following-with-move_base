import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, PoseStamped
from tf2_geometry_msgs import do_transform_pose

def transform_pose_to_map():
    # 创建tf Buffer
    tf_buffer = tf2_ros.Buffer()
    # 创建tf监听器
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 等待tf树中的转换关系建立好
    rospy.sleep(0.5)

    # 创建一个PoseWithCovarianceStamped消息，该消息包含物体在camera_link坐标系中的位置和方向
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = "tb3_0/camera_link"
    pose_msg.pose.pose = Pose(Point(0.5, 0, 0), Quaternion(0, 0, 0, 1))

    try:
        # 获取camera_link到map的变换关系
        transform = tf_buffer.lookup_transform("map", pose_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        
        # 使用tf2库将位姿信息从camera_link坐标系转换到map坐标系
        transformed_pose = do_transform_pose(pose_msg.pose, transform)

        # 创建一个PoseStamped消息，用于发布到/tb3_0/move_base_simple/goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = transformed_pose.pose

        # 创建一个Publisher，发布到/tb3_0/move_base_simple/goal
        pub = rospy.Publisher('/tb3_0/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(0.5)  # 等待Publisher建立连接
        pub.publish(goal_msg)

        # 处理转换后的位姿信息，可以根据需要进行后续操作
        print("Transformed Pose in map frame:")
        print(transformed_pose)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to transform pose: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('pose_transformer')
    transform_pose_to_map()
    rospy.spin()