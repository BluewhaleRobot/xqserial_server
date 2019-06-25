
#include <ros/ros.h>
#include "xqserial_server/RawOdom.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "tf/transform_broadcaster.h"

using namespace std;

class NewOdomPuber{
  public:
    NewOdomPuber() {
      mOdomPub = mNH.advertise<nav_msgs::Odometry>("/Odom_new", 1, true);
      raw_odom_last.data=0;
      CarPos2D.x = 0.0;
      CarPos2D.y = 0.0;
      CarPos2D.theta = 0.0;

      CarTwist.linear.x = 0.0;
      CarTwist.linear.y = 0.0;
      CarTwist.linear.z = 0.0;
      CarTwist.angular.x = 0.0;
      CarTwist.angular.y = 0.0;
      CarTwist.angular.z = 0.0;
    }
    void GrabImuAndRawOdom(const xqserial_server::RawOdomConstPtr& raw_odom_msg,const sensor_msgs::ImuConstPtr& imu_msg);
    ros::Publisher mOdomPub;
    ros::NodeHandle mNH;
    xqserial_server::RawOdom raw_odom_last;
    geometry_msgs::Pose2D CarPos2D; // 小车开始启动原点坐标系
    geometry_msgs::Twist CarTwist;  // 小车自身坐标系
    nav_msgs::Odometry CarOdom;
};

void NewOdomPuber::GrabImuAndRawOdom(const xqserial_server::RawOdomConstPtr& raw_odom_msg,const sensor_msgs::ImuConstPtr& imu_msg)
{
  static bool first_pack = true;
  if(first_pack)
  {
    raw_odom_last.header = raw_odom_msg->header;
    raw_odom_last.data = raw_odom_msg->data;
    first_pack = false;
  }

  double delta_x,delta_y,delta_car,delta_time;

  delta_car = raw_odom_msg->data - raw_odom_last.data;

  delta_x = delta_car * cos(CarPos2D.theta );
  delta_y = delta_car * sin(CarPos2D.theta );

  CarPos2D.x += delta_x;
  CarPos2D.y += delta_y;

  //处理角度
  tf::Matrix3x3 Rob(tf::Quaternion(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w));
  double roll, pitch, yaw;
  Rob.getRPY(roll, pitch, yaw);

  CarPos2D.theta = yaw;

  ros::Duration delta_time_now = raw_odom_msg->header.stamp - raw_odom_last.header.stamp;

  CarTwist.linear.x = delta_car * 1.0/delta_time_now.toSec();

  CarTwist.angular.z = imu_msg->angular_velocity.z*3.1415926/180.0;

  CarOdom.header.stamp = raw_odom_msg->header.stamp;
  CarOdom.header.frame_id = "odom_imu";
  CarOdom.pose.pose.position.x = CarPos2D.x;
  CarOdom.pose.pose.position.y = CarPos2D.y;
  CarOdom.pose.pose.position.z = 0.0f;
  CarOdom.pose.pose.orientation = imu_msg->orientation;
  CarOdom.child_frame_id = "base_footprint";
  CarOdom.twist.twist.linear.x = CarTwist.linear.x; // * cos(CarPos2D.theta* PI / 180.0f);
  CarOdom.twist.twist.linear.y = CarTwist.linear.y; // * sin(CarPos2D.theta* PI / 180.0f);
  CarOdom.twist.twist.angular.z = CarTwist.angular.z;
  mOdomPub.publish(CarOdom);

  raw_odom_last.header = raw_odom_msg->header;
  raw_odom_last.data = raw_odom_msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "new_odom");
    ros::start();

    ros::NodeHandle nh;

    NewOdomPuber new_odom_puber;

    message_filters::Subscriber<xqserial_server::RawOdom> raw_odom_sub(nh, "/xqserial_server/Raw_odom", 30);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/IMU", 30);
    typedef message_filters::sync_policies::ApproximateTime<xqserial_server::RawOdom, sensor_msgs::Imu> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(30), raw_odom_sub, imu_sub);
    auto f = boost::bind(&NewOdomPuber::GrabImuAndRawOdom, &new_odom_puber, _1, _2);
    sync.registerCallback(f);

    ros::spin();

    ros::shutdown();
    return 0;
}
