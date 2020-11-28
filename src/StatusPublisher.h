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
#include <stdint.h>
#include "crc16.h"

#define PI 3.14159265

namespace xqserial_server
{
typedef struct {
  int status;//小车状态，0表示未初始化，1表示正常，-1表示error
  float power;             //电源电压【9 13】v
  float theta_imu;             //方位角，【0 360）°
  int encoder_ppr_imu;         //车轮1转对应的编码器个数
  int encoder_delta_r;     //右轮编码器增量， 个为单位
  int encoder_delta_l;     //左轮编码器增量， 个为单位
  unsigned int upwoard;    //1表示正向安装,0表示反向安装
  unsigned int hbz_status; //红外模块状态，8421
  float sonar_distance[4]; //超声模块距离值 单位m
  float quat[4];          //IMU四元数
  float IMU[9];           //IMU 9轴数据
  unsigned int time_stamp_imu;//时间戳

  float theta;

  int encoder_ppr;
  int encoder_r_current;  //右轮编码器当期读数， 个为单位
  int encoder_l_current;  //左轮编码器当期读数， 个为单位
  int driver_error;      //0正常，正数代表驱动器错误状态
  int driver_mode1;      //0 其他模式， 1 速度模式
  int driver_mode2;      //0 其他模式， 1 速度模式
  int driver_status;       //0复位状态，1 can模式, 2 已进入can 速度模式
}UPLOAD_STATUS;

class StatusPublisher
{

public:
    StatusPublisher();
    StatusPublisher(double separation,double radius,double power_scale);
    void Refresh();
    void ResetDriver()
    {
      boost::mutex::scoped_lock lock(mMutex_car);
      car_status.driver_status = 0;
      car_status.driver_mode1 = 0;
      car_status.driver_mode2 = 0;
      car_status.encoder_r_current = 0;
      car_status.encoder_l_current = 0;
      encoder_r_last = 0;
      encoder_l_last = 0;
    }
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
    sensor_msgs::Range CarSonar3;
    sensor_msgs::Range CarSonar4;

    geometry_msgs::Pose2D CarPos2D;//小车开始启动原点坐标系
    geometry_msgs::Twist  CarTwist;//小车自身坐标系
    std_msgs::Float64 CarPower;// 小车电池信息
    nav_msgs::Odometry CarOdom;// 小车位置和速度信息

    ros::NodeHandle mNH;

    ros::Publisher mPose2DPub;
    ros::Publisher mTwistPub;
    ros::Publisher mStatusFlagPub;
    ros::Publisher mDriverFlagPub;
    ros::Publisher mPowerPub;
    ros::Publisher mOdomPub;

    ros::Publisher pub_barpoint_cloud_;
    ros::Publisher pub_clearpoint_cloud_;

    ros::Publisher mSonar1Pub;
    ros::Publisher mSonar2Pub;
    ros::Publisher mSonar3Pub;
    ros::Publisher mSonar4Pub;
    ros::Publisher mIMUPub;
    sensor_msgs::Imu  CarIMU;

    bool mbUpdated_imu;
    bool mbUpdated_car;
    double  distances_[4];

    boost::mutex mMutex_car;
    boost::mutex mMutex_imu;

    double power_scale_;

    double rot_dist_;
    double tran_dist_;

    bool forward_flag_;
    bool rot_flag_;

    int encoder_r_last;  //右轮编码器上期读数， 个为单位
    int encoder_l_last;  //左轮编码上期读数， 个为单位

    float yaw_deltas[100];
    int yaw_index;
    float yaw_sum;
    float yaw_omega;

    ros::WallTime last_sonartime_;
    float min_sonardist_;
};

} //namespace xqserial_server


#endif // STATUSPUBLISHER_H
