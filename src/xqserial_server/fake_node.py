#!/usr/bin/env python3
#encoding=utf-8

"""
这是一个小强驱动包的假节点。这个节点发布以下几个topic
/xqserial_server/Odom
/xqserial_server/StatusFlag
/xqserial_server/Pose2D
/xqserial_server/Power
位置数据全部为0， 电压数据12V
同时发布odom -> baselink 的 静态tf变换
"""

import rospy
from std_msgs.msg import Int32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


if __name__ == "__main__":
    rospy.init_node("fake_xqserial_server", anonymous=True)
    rate = rospy.Rate(50)
    odom_pub = rospy.Publisher("/xqserial_server/Odom", Odometry, queue_size=0)
    status_flag_pub = rospy.Publisher("/xqserial_server/StatusFlag", Int32,
        queue_size=0)
    pose_pub = rospy.Publisher("/xqserial_server/Pose2D", Pose2D, queue_size=0)
    power_pub = rospy.Publisher("/xqserial_server/Power", Float64, queue_size=0)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        mOdom = Odometry()
        mOdom.header.stamp = now
        mOdom.header.frame_id = "odom"
        mOdom.child_frame_id = "base_footprint"
        mOdom.pose.pose.orientation.x = 0
        mOdom.pose.pose.orientation.y = 0
        mOdom.pose.pose.orientation.z = 0
        mOdom.pose.pose.orientation.w = 1
        odom_pub.publish(mOdom)
        status = Int32()
        status.data = 1
        status_flag_pub.publish(status)
        mPose = Pose2D()
        pose_pub.publish(mPose)
        mPower = Float64()
        mPower.data = 11.0
        power_pub.publish(mPower)
        rate.sleep()
