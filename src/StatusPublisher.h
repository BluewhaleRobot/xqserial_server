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
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#define PI 3.14159265

namespace xqserial_server
{
typedef struct {
    int status;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power;//电源电压【9 13】v
    float theta;//方位角，【0 360）°
    unsigned int encoder_ppr;//车轮1转对应的编码器个数
    int encoder_delta_l;//左轮编码器增量， 个为单位
    int encoder_delta_r;//右轮编码器增量， 个为单位
    int encoder_delta_car;//两车轮中心位移，个为单位
    unsigned int  upwoard;//0表示正面朝下安装，１表示正面朝上安装
    float max_speed;//最大转速，圈每秒
    int hbz1;//通道１红外状态，０表示正常１表示触发
    int hbz2;//通道２红外状态，０表示正常１表示触发
    int hbz3;//通道３红外状态，０表示正常１表示触发
    int hbz4;//通道４红外状态，０表示正常１表示触发
    float distance1;//第一个超声模块距离值 单位cm
    float distance2;//第二个超声模块距离值 单位cm
    float IMU[9];//mpu9250 9轴数据
    unsigned int time_stamp;//时间戳
}UPLOAD_STATUS;

class StatusPublisher
{

public:
    StatusPublisher();
    StatusPublisher(double separation,double radius,double power_scale);
    void Refresh();
    void Update(const char *data, unsigned int len);
    double get_wheel_separation();
    double get_wheel_radius();
    int get_wheel_ppr();
    int get_status();
    geometry_msgs::Pose2D get_CarPos2D();
    void get_wheel_speed(double speed[2]);
    geometry_msgs::Twist get_CarTwist();
    std_msgs::Float64 get_power();
    nav_msgs::Odometry get_odom();
    UPLOAD_STATUS car_status;
    void get_distances(double distances[2]);
    void setBarParams(double rot_dist,double tran_dist)
    {
      rot_dist_ = rot_dist;
      tran_dist_ = tran_dist;
    }
    void get_canmove_flag(bool &forward_flag,bool &rot_flag);
    float get_ultrasonic_min_distance();
private:



    //Wheel separation, wrt the midpoint of the wheel width: meters
    double wheel_separation;

    //Wheel radius (assuming it's the same for the left and right wheels):meters
    double wheel_radius;

    sensor_msgs::Range CarSonar1;
    sensor_msgs::Range CarSonar2;

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
    ros::Publisher mSonar1Pub;
    ros::Publisher mSonar2Pub;
    ros::Publisher mIMUPub;
    sensor_msgs::Imu  CarIMU;

    bool mbUpdated;
    double  distances_[2];
    boost::mutex mMutex;

    double power_scale_;

    double rot_dist_;
    double tran_dist_;

    bool forward_flag_;
    bool rot_flag_;
};

} //namespace xqserial_server


#endif // STATUSPUBLISHER_H
