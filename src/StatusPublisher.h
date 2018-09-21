#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <algorithm>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#define PI 3.14159265

namespace xqserial_server
{
typedef struct {
    int status;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power;//电源电压【9 13】v
    float theta;//方位角，【0 360）°
    float roll;//方位角，【-180 180）°
    float pitch;//方位角，【-90 90）°
    unsigned int encoder_ppr;//车轮1转对应的编码器个数
    int encoder_delta_1;//1号编码器增量， 个为单位
    int encoder_delta_2;//1号编码器增量， 个为单位
    int encoder_delta_3;//1号编码器增量， 个为单位
    int encoder_delta_4;//1号编码器增量， 个为单位
    float distance[4];//超声模块距离值 单位m
    float IMU[9];//mpu9250 9轴数据
    unsigned int forward_switch;
    unsigned int backward_switch;
    unsigned int time_stamp;//时间戳
}UPLOAD_STATUS;

class StatusPublisher
{
public:
    StatusPublisher();
    StatusPublisher(double separation,double radius);
    void Refresh();
    void Update(const char *data, unsigned int len);
    double get_wheel_separation();
    double get_wheel_radius();
    int get_wheel_ppr();
    int get_status();
    geometry_msgs::Pose2D get_CarPos2D();
    geometry_msgs::Twist get_CarTwist();
    std_msgs::Float64 get_power();
    nav_msgs::Odometry get_odom();
    UPLOAD_STATUS car_status;
    void getSonarData(float (&ranges)[4],float (&view_angles)[4]);
private:



    //Wheel separation, wrt the midpoint of the wheel width: meters
    double wheel_separation;

    //Wheel radius (assuming it's the same for the left and right wheels):meters
    double wheel_radius;

    geometry_msgs::Pose2D CarPos2D;//小车开始启动原点坐标系
    geometry_msgs::Twist  CarTwist;//小车自身坐标系
    std_msgs::Float64 CarPower;// 小车电池信息
    nav_msgs::Odometry CarOdom;// 小车位置和速度信息
    ros::NodeHandle mNH;
    ros::Publisher mPose2DPub;
    ros::Publisher mTwistPub;
    ros::Publisher mStatusFlagPub;
    ros::Publisher mPowerPub;
    ros::Publisher mOdomPub;
    ros::Publisher pub_barpoint_cloud_;
    ros::Publisher pub_clearpoint_cloud_;

    sensor_msgs::Range CarSonar1;
    sensor_msgs::Range CarSonar2;
    sensor_msgs::Range CarSonar3;
    sensor_msgs::Range CarSonar4;

    ros::Publisher mSonar1Pub;
    ros::Publisher mSonar2Pub;
    ros::Publisher mSonar3Pub;
    ros::Publisher mSonar4Pub;

    bool mbUpdated;
    boost::mutex mMutex;
    double base_time_;

    ros::Publisher mIMUPub;
    sensor_msgs::Imu  CarIMU;
    float ranges_[4];
    float view_angles_[4];
};

} //namespace xqserial_server


#endif // STATUSPUBLISHER_H
