import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_geometry_msgs import do_transform_pose

def pose_callback(msg):
    # 创建tf Buffer
    tf_buffer = tf2_ros.Buffer()
    # 创建tf监听器
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 等待tf树中的转换关系建立好
    rospy.sleep(0.5)

    try:
        # 获取地图到机器人底盘的变换关系
        transform = tf_buffer.lookup_transform("base_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        
        # 使用tf2库将位姿信息从地图坐标系转换到机器人底盘坐标系
        transformed_pose = do_transform_pose(msg.pose, transform)

        # 处理转换后的位姿信息，可以根据需要进行后续操作
        print("Transformed Pose in base_link frame:")
        print(transformed_pose)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to transform pose: %s", str(e))

def main():
    rospy.init_node('pose_transformer')
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
