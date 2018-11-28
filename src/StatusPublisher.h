#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "ros/ros.h"
#include "ros/console.h"
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
#include <stdint.h>
#include "crc16.h"

#define PI 3.14159265

namespace xqserial_server
{
typedef struct {
    int status_imu;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power_imu;             //电源电压【9 13】v
    float theta;             //方位角，【0 360）°
    int encoder_ppr;         //车轮1转对应的编码器个数
    int encoder_delta_r;     //右轮编码器增量， 个为单位
    int encoder_delta_l;     //左轮编码器增量， 个为单位
    int encoder_delta_car;   //两车轮中心位移，个为单位
    int omga_r;              //右轮转速 个每秒
    int omga_l;              //左轮转速 个每秒
    float distance1;         //第一个超声模块距离值 单位cm
    float distance2;         //第二个超声模块距离值 单位cm
    float distance3;         //第三个超声模块距离值 单位cm
    float distance4;         //第四个超声模块距离值 单位cm
    float IMU[9];            //mpu9250 9轴数据
    unsigned int time_stamp_imu;//时间戳

    int encoder_r_current;//右轮编码器当期读数， 个为单位
    int encoder_l_current;//左轮编码器当期读数， 个为单位
    int left_rpm;// 3000.0/8192.0 RPM
    int right_rpm;//
    int left_driver_status;//
    int right_driver_status;//
    int power_left;  //V
    int power_right; //V
    int current_left; //0.01A
    int current_right; //0.01A
    unsigned int current_register;

    int status;

}UPLOAD_STATUS;

class StatusPublisher
{

public:
    StatusPublisher();
    StatusPublisher(double separation,double radius,bool debugFlag);
    void Refresh();
    void Update_car(const char *data, unsigned int len);
    void Update_imu(const char *data, unsigned int len);
    double get_wheel_separation();
    double get_wheel_radius();
    int get_wheel_ppr();
    int get_status();
    geometry_msgs::Pose2D get_CarPos2D();
    void get_wheel_speed(double speed[2]);
    geometry_msgs::Twist get_CarTwist();
    std_msgs::Float64 get_power();
    nav_msgs::Odometry get_odom();
    void set_register(unsigned int address);
    UPLOAD_STATUS car_status;
private:



    //Wheel separation, wrt the midpoint of the wheel width: meters
    double wheel_separation;

    //Wheel radius (assuming it's the same for the left and right wheels):meters
    double wheel_radius;

    geometry_msgs::Pose2D CarPos2D;//小车开始启动原点坐标系
    geometry_msgs::Twist  CarTwist;//小车自身坐标系
    std_msgs::Float64 CarPower;// 小车电池信息
    nav_msgs::Odometry CarOdom;// 小车位置和速度信息
    sensor_msgs::Imu  CarIMU;

    ros::NodeHandle mNH;
    ros::Publisher mPose2DPub;
    ros::Publisher mTwistPub;
    ros::Publisher mStatusFlagPub;
    ros::Publisher mPowerPub;
    ros::Publisher mOdomPub;
    ros::Publisher mIMUPub;

    bool mbUpdated_imu;
    bool mbUpdated_car;

    boost::mutex mMutex_car;
    boost::mutex mMutex_imu;

    float yaw_deltas[100];
    int yaw_index;
    float yaw_sum;
    float yaw_omega;
    bool yaw_ready;

    bool debug_flag;
    double base_time_;
};

} //namespace xqserial_server


#endif // STATUSPUBLISHER_H
