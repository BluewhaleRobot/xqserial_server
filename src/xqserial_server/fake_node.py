#!/usr/bin/env python
# encoding=utf-8

"""
这是一个小强驱动包的假节点。这个节点发布以下几个topic
/kinect/clearpoints
/xqserial_server/IMU
/xqserial_server/Odom
/xqserial_server/StatusFlag
/xqserial_server/Pose2D
/xqserial_server/Power
/xqserial_server/Twist
位置数据全部为0， 电压数据12V
同时发布odom -> base_footprint 的tf变换
订阅消息 
/cmd_vel
/global_move_flag
/xqserial_server/initPose
"""

import threading
import rospy
from geometry_msgs.msg import Pose2D, Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int32, Bool
from sensor_msgs.msg import Imu
import tf
from tf.transformations import quaternion_from_euler
import math

CURRENT_TWIST = Twist()
MOVE_FLAG = Bool()
MOVE_FLAG.data = True
CURRENT_ODOM = Odometry()
CURRENT_POSE2D = Pose2D()
DATA_LOCK = threading.RLock()
CURRENT_IMU = Imu()

def update_current_speed(twist):
    global CURRENT_TWIST, DATA_LOCK
    with DATA_LOCK:
        CURRENT_TWIST.linear.x = twist.linear.x
        CURRENT_TWIST.linear.y = twist.linear.y
        CURRENT_TWIST.linear.z = twist.linear.z
        CURRENT_TWIST.angular.x = twist.angular.x
        CURRENT_TWIST.angular.y = twist.angular.y
        CURRENT_TWIST.angular.z = twist.angular.z

def update_move_flag(flag):
    global MOVE_FLAG, DATA_LOCK
    with DATA_LOCK:
        MOVE_FLAG.data = flag.data

def update_location():
    global DATA_LOCK, CURRENT_ODOM, MOVE_FLAG, CURRENT_TWIST, CURRENT_POSE2D, CURRENT_IMU
    rate = rospy.Rate(50)
    odom_pub = rospy.Publisher("/xqserial_server/Odom", Odometry, queue_size=0)
    pose_pub = rospy.Publisher("/xqserial_server/Pose2D", Pose2D, queue_size=0)
    twist_pub = rospy.Publisher("/xqserial_server/Twist", Twist, queue_size=0)
    power_pub = rospy.Publisher(
        "/xqserial_server/Power", Float64, queue_size=0)
    imu_pub = rospy.Publisher("/xqserial_server/IMU", Imu, queue_size=0)
    status_flag_pub = rospy.Publisher("/xqserial_server/StatusFlag", Int32,
        queue_size=0)
    odom_tf = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        with DATA_LOCK:
            if MOVE_FLAG.data:
                # 更新当前位置
                CURRENT_POSE2D.x += CURRENT_TWIST.linear.x * 0.02 * math.cos( CURRENT_POSE2D.theta * math.pi / 180 )
                CURRENT_POSE2D.y += CURRENT_TWIST.linear.x * 0.02 * math.sin( CURRENT_POSE2D.theta * math.pi / 180 )
                CURRENT_POSE2D.theta += CURRENT_TWIST.angular.z * 0.02 * 180 / math.pi
                if CURRENT_POSE2D.theta > 360:
                    CURRENT_POSE2D.theta -= 360
                if CURRENT_POSE2D.theta < 0:
                    CURRENT_POSE2D.theta += 360
                pose_pub.publish(CURRENT_POSE2D)
                # 更新odom
                CURRENT_ODOM.header.stamp = rospy.Time.now()
                CURRENT_ODOM.header.frame_id = "odom"
                CURRENT_ODOM.pose.pose.position.x = CURRENT_POSE2D.x
                CURRENT_ODOM.pose.pose.position.y = CURRENT_POSE2D.y
                CURRENT_ODOM.pose.pose.position.z = 0
                q_angle = quaternion_from_euler(0, 0, CURRENT_POSE2D.theta * 3.14 / 180, axes='sxyz')
                q = Quaternion(*q_angle)
                CURRENT_ODOM.pose.pose.orientation = q
                CURRENT_ODOM.twist.twist = CURRENT_TWIST
                odom_pub.publish(CURRENT_ODOM)
            else:
                pose_pub.publish(CURRENT_POSE2D)
                CURRENT_ODOM.header.stamp = rospy.Time.now()
                CURRENT_ODOM.header.frame_id = "odom"
                odom_pub.publish(CURRENT_ODOM)
        # 更新IMU数据
        CURRENT_IMU.header.frame_id = "imu"
        CURRENT_IMU.header.stamp = rospy.Time.now()
        q_angle = quaternion_from_euler(0, 0, CURRENT_POSE2D.theta * 3.14 / 180, axes='sxyz')
        q = Quaternion(*q_angle)
        CURRENT_IMU.orientation = q
        CURRENT_IMU.angular_velocity.x = 0
        CURRENT_IMU.angular_velocity.y = 0
        CURRENT_IMU.angular_velocity.z = CURRENT_TWIST.angular.z
        CURRENT_IMU.linear_acceleration.x = 0
        CURRENT_IMU.linear_acceleration.y = 0
        CURRENT_IMU.linear_acceleration.z = 9.8
        imu_pub.publish(CURRENT_IMU)
        # 更新电压
        current_power = Float64()
        current_power.data = 12.0    
        power_pub.publish(current_power)
        # 更新statusFlag
        current_status = Int32()
        current_status.data = 0
        status_flag_pub.publish(current_status)
        # 发布 Twist
        twist_pub.publish(CURRENT_TWIST)
        # 发布 base_footprint到odom的tf
        odom_tf.sendTransform((CURRENT_POSE2D.x, CURRENT_POSE2D.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, CURRENT_POSE2D.theta * math.pi / 180),
            rospy.Time.now(),
            "base_footprint",
            "odom"
        )
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("xqserial_server", anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, update_current_speed)
    rospy.Subscriber("/global_move_flag", Bool, update_move_flag)
    update_location()
    
    
    
