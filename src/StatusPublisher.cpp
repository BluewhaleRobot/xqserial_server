#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>

#define DISABLE 0
#define ENABLE 1

namespace xqserial_server
{
typedef sensor_msgs::PointCloud2 PointCloud;

StatusPublisher::StatusPublisher()
{
  mbUpdated = false;
  wheel_separation = 0.37;
  wheel_radius = 0.06;

  CarPos2D.x = 0.0;
  CarPos2D.y = 0.0;
  CarPos2D.theta = 0.0;

  CarTwist.linear.x = 0.0;
  CarTwist.linear.y = 0.0;
  CarTwist.linear.z = 0.0;
  CarTwist.angular.x = 0.0;
  CarTwist.angular.y = 0.0;
  CarTwist.angular.z = 0.0;

  CarPower.data = 0.0;

  int i = 0;
  int *status;
  status = (int *)&car_status;
  for (i = 0; i < 23; i++)
  {
    status[i] = 0;
  }
  car_status.encoder_ppr = 4 * 12 * 64;

  mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D", 1, true);
  mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag", 1, true);
  mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist", 1, true);
  mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
  mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);
  mOdomPub2 = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom2", 1, true);
  mOdomPub3 = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom3", 1, true);
  pub_barpoint_cloud_ = mNH.advertise<PointCloud>("kinect/barpoints", 1, true);
  pub_clearpoint_cloud_ = mNH.advertise<PointCloud>("kinect/clearpoints", 1, true);
  mIMUPub = mNH.advertise<sensor_msgs::Imu>("xqserial_server/IMU", 1, true);
  /* static tf::TransformBroadcaster br;
   tf::Quaternion q;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.13) );//摄像头距离地面高度13cm
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
   */
  base_time_ = ros::Time::now().toSec();

  msr = 0;
  msl = 0;
  mtheta = 0;
  mt = 0;
  mvtheta = 0;

  mx_raw = 0;
  my_raw = 0;
  mtheta_raw = 0;

  mx_raw3 = 0;
  my_raw3 = 0;
  mtheta_raw3 = 0;

  Jf = cv::Mat::zeros(8, 8, CV_32F); //8x8
  Jh = cv::Mat::zeros(4, 8, CV_32F); //4x8
  x0 = cv::Mat::zeros(8, 1, CV_32F); //8x1
  Pk = cv::Mat::zeros(8, 8, CV_32F); //8x8
  Qk = cv::Mat::zeros(8, 8, CV_32F); //8x8
  Rk = cv::Mat::zeros(4, 4, CV_32F); //4x4
  Xka = cv::Mat::zeros(8, 1, CV_32F); //8x1
  Zk = cv::Mat::zeros(4, 1, CV_32F); //4x1

}

StatusPublisher::StatusPublisher(double separation, double radius)
{
  new (this) StatusPublisher();
  wheel_separation = separation;
  wheel_radius = radius;

  float dt = 0.02;
  float half_dt2 = 0.0002;
  Jf.at<float>(0,2) = dt;
  Jf.at<float>(0,4) = half_dt2;
  Jf.at<float>(1,3) = dt;
  Jf.at<float>(1,5) = half_dt2;
  Jf.at<float>(2,2) = 1;
  Jf.at<float>(2,4) = dt;
  Jf.at<float>(3,3) = 1;
  Jf.at<float>(3,5) = dt;
  Jf.at<float>(4,4) = 1;
  Jf.at<float>(5,5) = 1;
  Jf.at<float>(6,6) = 1;
  Jf.at<float>(7,7) = 1;

  x0.at<float>(7) = 0;

  //Pk.at<float>(6,6) = 0.001;
  //Pk.at<float>(7,7) = 0.1;

  Qk.at<float>(0,0) = 0.0001;
  Qk.at<float>(1,1) = 0.0001;
  Qk.at<float>(2,2) = 0.0001;
  Qk.at<float>(3,3) = 0.0001;
  Qk.at<float>(4,4) = 0.01;
  Qk.at<float>(5,5) = 0.01;
  Qk.at<float>(6,6) = 0.0001;
  Qk.at<float>(7,7) = 0.001;

  Rk.at<float>(0,0) = 2;
  Rk.at<float>(1,1) = 2;
  Rk.at<float>(2,2) = 0.000001;
  Rk.at<float>(3,3) = 0.0001;

  Xka = x0.clone();

  std::stringstream buf_Xka;
  buf_Xka <<"Xka "<< Xka<<std::endl;
  ROS_ERROR("%s",buf_Xka.str().c_str());

  std::stringstream buf_Qk;
  buf_Qk <<"Qk "<< Qk<<std::endl;
  ROS_ERROR("%s",buf_Qk.str().c_str());

  std::stringstream buf_Rk;
  buf_Rk <<"Rk "<< Rk<<std::endl;
  ROS_ERROR("%s",buf_Rk.str().c_str());
}

void StatusPublisher::Update(const char data[], unsigned int len)
{
  // if(len <1) return;
  // static char data2[1024];
  // static int len2=0;
  boost::mutex::scoped_lock lock(mMutex);

  int i = 0, j = 0;
  int *receive_byte;
  static unsigned char last_str[2] = {0x00, 0x00};
  static unsigned char new_packed_ctr = DISABLE; //ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
  static int new_packed_ok_len = 0;              //包的理论长度
  static int new_packed_len = 0;                 //包的实际长度
  static unsigned char cmd_string_buf[512];
  unsigned char current_str = 0x00;
  const int cmd_string_max_size = 512;
  receive_byte = (int *)&car_status;
  //int ii=0;
  //boost::mutex::scoped_lock lock(mMutex);

  // if(len<119)
  // {
  // std::cout<<"len0:"<<len<<std::endl;
  //   current_str=data[0];
  //   std::cout<<(unsigned int)current_str<<std::endl;
  // }
  for (i = 0; i < len; i++)
  {
    current_str = data[i];
    // unsigned int temp=(unsigned int)current_str;
    // std::cout<<temp<<std::endl;
    //判断是否有新包头
    if (last_str[0] == 205 && last_str[1] == 235 && current_str == 215) //包头 205 235 215
    {
      //std::cout<<"runup1 "<<std::endl;
      new_packed_ctr = ENABLE;
      new_packed_ok_len = 0;
      new_packed_len = new_packed_ok_len;
      last_str[0] = last_str[1]; //保存最后两个字符，用来确定包头
      last_str[1] = current_str;
      continue;
    }
    last_str[0] = last_str[1]; //保存最后两个字符，用来确定包头
    last_str[1] = current_str;
    if (new_packed_ctr == ENABLE)
    {

      //获取包长度
      new_packed_ok_len = current_str;
      if (new_packed_ok_len > cmd_string_max_size)
        new_packed_ok_len = cmd_string_max_size; //包内容最大长度有限制
      new_packed_ctr = DISABLE;
      //std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
    }
    else
    {
      //判断包当前大小
      if (new_packed_ok_len <= new_packed_len)
      {
        //std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        //包长度已经大于等于理论长度，后续内容无效
        continue;
      }
      else
      {
        //获取包内容
        new_packed_len++;
        cmd_string_buf[new_packed_len - 1] = current_str;
        if (new_packed_ok_len == new_packed_len && new_packed_ok_len > 0)
        {
          // std::cout<<"runup4 "<<std::endl;
          //当前包已经处理完成，开始处理
          if (new_packed_ok_len == 115)
          {
            for (j = 0; j < 23; j++)
            {
              memcpy(&receive_byte[j], &cmd_string_buf[5 * j], 4);
            }
            mbUpdated = true;
          }
          else if (new_packed_ok_len == 95)
          {
            for (j = 0; j < 19; j++)
            {
              memcpy(&receive_byte[j], &cmd_string_buf[5 * j], 4);
            }
            mbUpdated = true;
          }
          if (mbUpdated)
          {
            for (j = 0; j < 7; j++)
            {
              if (cmd_string_buf[5 * j + 4] != 32)
              {
                //   std::cout<<"len:"<< len <<std::endl;
                //   std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
                //   for(j=0;j<115;j++)
                //   {
                //     current_str=cmd_string_buf[j];
                //     std::cout<<(unsigned int)current_str<<std::endl;
                //   }
                mbUpdated = false;
                car_status.encoder_ppr = 4 * 12 * 64;
                break;
              }
            }
          }
          if (mbUpdated)
          {
            base_time_ = ros::Time::now().toSec();
          }
          // if(mbUpdated&&(car_status.encoder_delta_car>3000||car_status.encoder_delta_car<-3000))
          // {
          //   std::cout<<"len:"<< len <<std::endl;
          //   std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
          //   for(j=0;j<115;j++)
          //   {
          //     current_str=cmd_string_buf[j];
          //     std::cout<<(unsigned int)current_str<<std::endl;
          //   }
          //   std::cout<<"last len:"<<len2<<std::endl;
          //   for(j=0;j<len2;j++)
          //   {
          //     current_str=data2[j];
          //     std::cout<<(unsigned int)current_str<<std::endl;
          //   }
          //   std::cout<<"current:"<<std::endl;
          //   for(j=0;j<len;j++)
          //   {
          //     current_str=data[j];
          //     std::cout<<(unsigned int)current_str<<std::endl;
          //   }
          // }

          //ii++;
          //std::cout << ii << std::endl;
          new_packed_ok_len = 0;
          new_packed_len = 0;
        }
      }
    }
  }
  // for(j=0;j<len;j++)
  // {
  //   len2++;
  //   if(len2==1024) len2=1;
  //   data2[len2-1]=data[j];
  // }

  return;
}

void StatusPublisher::Refresh()
{
  boost::mutex::scoped_lock lock(mMutex);
  static double theta_last = 0.0;
  static unsigned int ii = 0;
  static bool theta_updateflag = false;
  ii++;
  //std::cout<<"runR"<< mbUpdated<<std::endl;
  if (mbUpdated)
  {
    // Time
    ros::Time current_time;

    if (car_status.status == 0)
    {
      theta_updateflag = false;
    }
    else
    {
      theta_updateflag = true;
    }
    //pose
    double delta_car, delta_x, delta_y, delta_theta, var_len, var_angle;

    var_len = (50.0f / car_status.encoder_ppr * 2.0f * PI * wheel_radius) * (50.0f / car_status.encoder_ppr * 2.0f * PI * wheel_radius);
    var_angle = (0.01f / 180.0f * PI) * (0.01f / 180.0f * PI);

    delta_car = (car_status.encoder_delta_r + car_status.encoder_delta_l) / 2.0f * 1.0f / car_status.encoder_ppr * 2.0f * PI * wheel_radius;
    if (delta_car > 0.05 || delta_car < -0.05)
    {
      // std::cout<<"get you!"<<std::endl;
      delta_car = 0;
    }
    // if(ii%50==0||car_status.encoder_delta_car>3000||car_status.encoder_delta_car<-3000)
    // {
    //   std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
    //   std::cout<<"delta_encoder_r:"<< car_status.encoder_delta_r <<std::endl;
    //   std::cout<<"delta_encoder_l:"<< car_status.encoder_delta_l <<std::endl;
    //   std::cout<<"ppr:"<< car_status.encoder_ppr <<std::endl;
    //   std::cout<<"delta_car:"<< delta_car <<std::endl;
    // }
    delta_x = delta_car * cos(CarPos2D.theta * PI / 180.0f);
    delta_y = delta_car * sin(CarPos2D.theta * PI / 180.0f);

    delta_theta = car_status.theta - theta_last;
    theta_last = car_status.theta;
    if (delta_theta > 270)
      delta_theta -= 360;
    if (delta_theta < -270)
      delta_theta += 360;

    if ((!theta_updateflag) || delta_theta > 20 || delta_theta < -20)
    {
      delta_theta = 0;
    }
    CarPos2D.x += delta_x;
    CarPos2D.y += delta_y;
    CarPos2D.theta += delta_theta;

    if (CarPos2D.theta > 360.0)
      CarPos2D.theta -= 360;
    if (CarPos2D.theta < 0.0)
      CarPos2D.theta += 360;

    mPose2DPub.publish(CarPos2D);

    //flag
    std_msgs::Int32 flag;
    flag.data = car_status.status;
    //底层障碍物信息
    if ((car_status.distance1 + car_status.distance2 + car_status.distance3 + car_status.distance4) > 0.1 && (car_status.distance1 + car_status.distance2 + car_status.distance3 + car_status.distance4) < 5.0)
    {
      //有障碍物
      flag.data = 2;
    }
    mStatusFlagPub.publish(flag);

    int barArea_nums = 0;
    int clearArea_nums = 0;
    if (car_status.distance1 > 0.1)
    {
      barArea_nums += 3;
    }
    else
    {
      clearArea_nums += 6;
    }
    if (car_status.distance2 > 0.1)
    {
      barArea_nums += 3;
    }
    else
    {
      clearArea_nums += 6;
    }
    if (car_status.distance4 > 0.1)
    {
      barArea_nums += 3;
    }
    else
    {
      clearArea_nums += 6;
    }

    if (barArea_nums > 0)
    {
      //发布雷区
      PointCloud::Ptr barcloud_msg(new PointCloud);
      barcloud_msg->header.stamp = current_time.fromSec(base_time_);
      barcloud_msg->height = 1;
      barcloud_msg->width = barArea_nums;
      barcloud_msg->is_dense = true;
      barcloud_msg->is_bigendian = false;
      barcloud_msg->header.frame_id = "kinect_link_new";
      sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
      pcd_modifier1.setPointCloud2FieldsByString(1, "xyz");
      sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
      if (car_status.distance2 > 0.1)
      {
        for (int k = 0; k < 3; k++, ++bariter_x, ++bariter_y, ++bariter_z)
        {
          *bariter_x = 0.2;
          *bariter_y = -0.10 - k * 0.05;
          *bariter_z = 0.15;
        }
      }
      if (car_status.distance4 > 0.1)
      {
        for (int k = 0; k < 3; k++, ++bariter_x, ++bariter_y, ++bariter_z)
        {
          *bariter_x = 0.2;
          *bariter_y = -0.1 + k * 0.05;
          *bariter_z = 0.15;
        }
      }
      if (car_status.distance1 > 0.1)
      {
        for (int k = 0; k < 3; k++, ++bariter_x, ++bariter_y, ++bariter_z)
        {
          *bariter_x = 0.2;
          *bariter_y = 0.05 + k * 0.05;
          *bariter_z = 0.15;
        }
      }
      if (ii % 5 == 0)
      {
        pub_barpoint_cloud_.publish(barcloud_msg);
      }
    }
    if (clearArea_nums > 0)
    {
      //发布雷区
      PointCloud::Ptr clearcloud_msg(new PointCloud);
      clearcloud_msg->header.stamp = current_time.fromSec(base_time_);
      clearcloud_msg->height = 1;
      clearcloud_msg->width = clearArea_nums;
      clearcloud_msg->is_dense = true;
      clearcloud_msg->is_bigendian = false;
      clearcloud_msg->header.frame_id = "kinect_link_new";
      sensor_msgs::PointCloud2Modifier pcd_modifier1(*clearcloud_msg);
      pcd_modifier1.setPointCloud2FieldsByString(1, "xyz");
      sensor_msgs::PointCloud2Iterator<float> cleariter_x(*clearcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> cleariter_y(*clearcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> cleariter_z(*clearcloud_msg, "z");
      if (car_status.distance2 < 0.1)
      {
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.2;
          *cleariter_y = -0.1 - k * 0.05;
          *cleariter_z = 0.0;
        }
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.15;
          *cleariter_y = -0.1 - k * 0.05;
          *cleariter_z = 0.0;
        }
      }
      if (car_status.distance4 < 0.1)
      {
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.2;
          *cleariter_y = -0.1 + k * 0.05;
          *cleariter_z = 0.0;
        }
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.15;
          *cleariter_y = -0.1 + k * 0.05;
          *cleariter_z = 0.0;
        }
      }
      if (car_status.distance1 < 0.1)
      {
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.2;
          *cleariter_y = 0.05 + k * 0.05;
          *cleariter_z = 0.0;
        }
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.15;
          *cleariter_y = 0.05 + k * 0.05;
          *cleariter_z = 0.0;
        }
      }
      if (ii % 5 == 0)
      {
        pub_clearpoint_cloud_.publish(clearcloud_msg);
      }
    }

    //Twist
    double angle_speed;
    CarTwist.linear.x = delta_car * 50.0f;
    angle_speed = -car_status.IMU[5];
    CarTwist.angular.z = angle_speed * PI / 180.0f;
    mTwistPub.publish(CarTwist);

    CarPower.data = car_status.power;
    mPowerPub.publish(CarPower);

    CarOdom.header.stamp = current_time.fromSec(base_time_);
    CarOdom.header.frame_id = "odom";
    CarOdom.pose.pose.position.x = CarPos2D.x;
    CarOdom.pose.pose.position.y = CarPos2D.y;
    CarOdom.pose.pose.position.z = 0.0f;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(CarPos2D.theta / 180.0f * PI);
    CarOdom.pose.pose.orientation = odom_quat;
    CarOdom.pose.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    CarOdom.child_frame_id = "base_footprint";
    CarOdom.twist.twist.linear.x = CarTwist.linear.x; // * cos(CarPos2D.theta* PI / 180.0f);
    CarOdom.twist.twist.linear.y = CarTwist.linear.y; // * sin(CarPos2D.theta* PI / 180.0f);
    CarOdom.twist.twist.angular.z = CarTwist.angular.z;
    CarOdom.twist.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    mOdomPub.publish(CarOdom);

    //publish IMU
    tf::Quaternion q_imu;
    q_imu.setRPY(0, 0, car_status.theta / 180.0 * PI);
    CarIMU.header.stamp = current_time;
    CarIMU.header.frame_id = "imu";
    CarIMU.orientation.x = q_imu.x();
    CarIMU.orientation.y = q_imu.y();
    CarIMU.orientation.z = q_imu.z();
    CarIMU.orientation.w = q_imu.w();

    CarIMU.angular_velocity.x = -car_status.IMU[3] * PI / 180.0f;
    CarIMU.angular_velocity.y = car_status.IMU[4] * PI / 180.0f;
    CarIMU.angular_velocity.z = -car_status.IMU[5] * PI / 180.0f;
    CarIMU.linear_acceleration.x = -car_status.IMU[0];
    CarIMU.linear_acceleration.y = car_status.IMU[1];
    CarIMU.linear_acceleration.z = -car_status.IMU[2];

    mIMUPub.publish(CarIMU);

    // pub transform

    static tf::TransformBroadcaster br;
    tf::Quaternion q;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(CarPos2D.x, CarPos2D.y, 0.0));
    q.setRPY(0, 0, CarPos2D.theta / 180 * PI);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, current_time.fromSec(base_time_), "odom", "base_footprint"));

    //ros::spinOnce();
    update_current_counters();
    //不用imu的里程计
    static int delta_counter = 0;
    float delta_x2 = delta_car * cos(mtheta_raw );
    float delta_y2 = delta_car * sin(mtheta_raw );

    float delta_theta2 = 0.2191*(1.0f*car_status.encoder_delta_r / car_status.encoder_ppr *2*PI - 1.0f*car_status.encoder_delta_l / car_status.encoder_ppr *2*PI);

    delta_counter += car_status.encoder_delta_r - car_status.encoder_delta_l;
    //ROS_ERROR("delta_counter %d %d",delta_counter,(car_status.encoder_delta_r - car_status.encoder_delta_l));

    if (delta_theta2 > PI)
      delta_theta2 -= 2*PI;
    if (delta_theta2 < -PI)
      delta_theta2 += 2*PI;

    mx_raw += delta_x2;
    my_raw += delta_y2;
    mtheta_raw += delta_theta2;

    CarOdom2.header.stamp = current_time.fromSec(base_time_);
    CarOdom2.header.frame_id = "odom";
    CarOdom2.pose.pose.position.x = mx_raw;
    CarOdom2.pose.pose.position.y = my_raw;
    CarOdom2.pose.pose.position.z = 0.0f;
    geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(mtheta_raw);
    CarOdom2.pose.pose.orientation = odom_quat2;
    CarOdom2.pose.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    CarOdom2.child_frame_id = "base_footprint";
    CarOdom2.twist.twist.linear.x = CarTwist.linear.x; // * cos(CarPos2D.theta* PI / 180.0f);
    CarOdom2.twist.twist.linear.y = CarTwist.linear.y; // * sin(CarPos2D.theta* PI / 180.0f);
    CarOdom2.twist.twist.angular.z = CarTwist.angular.z;
    CarOdom2.twist.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    mOdomPub2.publish(CarOdom2);

    // //考虑二阶小量的里程计
    // float delta_car_x = delta_car;
    // float delta_car_y = delta_car*delta_theta* PI / 180.0/2.0;
    //
    // float theta_temp = (CarPos2D.theta - delta_theta) * PI / 180.0f;
    // float delta_x2 = delta_car_x * cos(theta_temp) - sin(theta_temp)*delta_car_y;
    // float delta_y2 = delta_car_x * sin(theta_temp ) + cos(theta_temp)*delta_car_y;
    //
    // mx_raw += delta_x2;
    // my_raw += delta_y2;
    //
    // static float sum_delta_y = 0;
    // sum_delta_y += delta_car_y;
    // //ROS_ERROR("sum_delta_y %f %f",sum_delta_y, delta_car_y);
    //
    // CarOdom2.header.stamp = current_time.fromSec(base_time_);
    // CarOdom2.header.frame_id = "odom";
    // CarOdom2.pose.pose.position.x = mx_raw;
    // CarOdom2.pose.pose.position.y = my_raw;
    // CarOdom2.pose.pose.position.z = 0.0f;
    // geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(CarPos2D.theta * PI / 180.0f);
    // CarOdom2.pose.pose.orientation = odom_quat2;
    // CarOdom2.pose.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    // CarOdom2.child_frame_id = "base_footprint";
    // CarOdom2.twist.twist.linear.x = CarTwist.linear.x; // * cos(CarPos2D.theta* PI / 180.0f);
    // CarOdom2.twist.twist.linear.y = CarTwist.linear.y; // * sin(CarPos2D.theta* PI / 180.0f);
    // CarOdom2.twist.twist.angular.z = CarTwist.angular.z;
    // CarOdom2.twist.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    // mOdomPub2.publish(CarOdom2);

    //ekf融合的里程计
    Zk.at<float>(0) = car_status.encoder_delta_r;
    Zk.at<float>(1) = car_status.encoder_delta_l;
    Zk.at<float>(2) = delta_theta* PI / 180.0f;
    Zk.at<float>(3) = CarTwist.angular.z;

    do_ekf();
    float delta_car3 = Xka.at<float>(0);
    float delta_theta3 = Xka.at<float>(1);

    float delta_x3 = delta_car3 * cos(mtheta_raw3 );
    float delta_y3 = delta_car3 * sin(mtheta_raw3 );

    mx_raw3 += delta_x3;
    my_raw3 += delta_y3;

    mtheta_raw3 += delta_theta3;

    ROS_ERROR("delta %f %f %f %f %f %f %f %f",delta_car, delta_car3, delta_theta/180.0f*PI, delta_theta3,Xka.at<float>(6),Xka.at<float>(7),Xka.at<float>(2),Xka.at<float>(3));

    CarOdom3.header.stamp = current_time.fromSec(base_time_);
    CarOdom3.header.frame_id = "odom";
    CarOdom3.pose.pose.position.x = mx_raw3;
    CarOdom3.pose.pose.position.y = my_raw3;
    CarOdom3.pose.pose.position.z = 0.0f;
    geometry_msgs::Quaternion odom_quat3 = tf::createQuaternionMsgFromYaw(mtheta_raw3);
    CarOdom3.pose.pose.orientation = odom_quat3;
    CarOdom3.pose.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    CarOdom3.child_frame_id = "base_footprint";
    CarOdom3.twist.twist.linear.x = Xka.at<float>(2); // * cos(CarPos2D.theta* PI / 180.0f);
    CarOdom3.twist.twist.linear.y = CarTwist.linear.y; // * sin(CarPos2D.theta* PI / 180.0f);
    CarOdom3.twist.twist.angular.z = Xka.at<float>(3);
    CarOdom3.twist.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    mOdomPub3.publish(CarOdom3);

    mbUpdated = false;
  }
}

void StatusPublisher::do_ekf()
{
  //predictor
  cv::Mat Xkf = Jf*Xka;
  cv::Mat Pk1 = Pk.clone();
  cv::Mat Pkf = Jf*Pk1*Jf.t() + Qk;
  //corrector
  float multi_value = car_status.encoder_ppr/2.0f/PI/wheel_radius;
  float L = wheel_radius/0.2191f/2.0f;
  float dt = 0.02;
  cv::Mat Kk =  cv::Mat::zeros(8, 4, CV_32F);
  static int num_i = 0;
  num_i ++;
  for(int i =0;i<2;i++)
  {
    Jh.at<float>(0,0) = multi_value;
    Jh.at<float>(0,1) = multi_value*(1+Xkf.at<float>(7))*L;
    Jh.at<float>(0,7) = multi_value*Xkf.at<float>(1)*L;
    Jh.at<float>(1,0) = multi_value;
    Jh.at<float>(1,1) = -multi_value*(1+Xkf.at<float>(7))*L;
    Jh.at<float>(1,7) = -multi_value*Xkf.at<float>(1)*L;
    Jh.at<float>(2,1) = 1;
    Jh.at<float>(2,6) = 0;//dt;
    Jh.at<float>(3,3) = 1;
    Jh.at<float>(3,6) = 1;

    cv::Mat Zkf = cv::Mat::zeros(4, 1, CV_32F); //4x1
    Zkf.at<float>(0) = multi_value*((1+Xkf.at<float>(7))*L*Xkf.at<float>(1) + Xkf.at<float>(0));
    Zkf.at<float>(1) = multi_value*(-(1+Xkf.at<float>(7))*L*Xkf.at<float>(1) + Xkf.at<float>(0));
    Zkf.at<float>(2) = Xkf.at<float>(1);//Xkf.at<float>(1) + dt*Xkf.at<float>(6);
    Zkf.at<float>(3) = Xkf.at<float>(3) + Xkf.at<float>(6);

    cv::Mat temp_mat = Jh*Pkf*Jh.t() + Rk;
    Kk = Pkf*Jh.t()*temp_mat.inv();

    Xkf = Xkf + Kk*(Zk -Zkf);
    if(num_i % 10 ==0)
    {
      std::stringstream buf_Xkf;
      buf_Xkf << "ikf " << i <<" Xkf "<< Xkf<<std::endl;
      //ROS_ERROR("%s",buf_Xkf.str().c_str());

      std::stringstream buf_Zk;
      buf_Zk << "zk " << Zk <<std::endl;
      //ROS_ERROR("%s",buf_Zk.str().c_str());

      std::stringstream buf_Zkf;
      buf_Zkf << "zkf " << Zkf <<std::endl;
      //ROS_ERROR("%s",buf_Zkf.str().c_str());

      std::stringstream buf_Kk;
      buf_Kk << "Kk " << Kk <<std::endl;
      //ROS_ERROR("%s",buf_Kk.str().c_str());
    }

  }
  Xka = Xkf.clone();
  Pk = (cv::Mat::eye(8, 8, CV_32F) - Kk*Jh)*Pkf;
  // if(num_i % 10 ==0)
  // {
  //   std::stringstream buf_Pk;
  //   buf_Pk <<"Pk "<< Pk<<std::endl;
  //   ROS_ERROR("%s",buf_Pk.str().c_str());
  // }
}

double StatusPublisher::get_wheel_separation()
{
  return wheel_separation;
}

double StatusPublisher::get_wheel_radius()
{
  return wheel_radius;
}

int StatusPublisher::get_wheel_ppr()
{
  return car_status.encoder_ppr;
}

void StatusPublisher::get_wheel_speed(double speed[2])
{
  //右一左二
  speed[0] = car_status.omga_r / car_status.encoder_ppr * 2.0 * PI * wheel_radius;
  speed[1] = car_status.omga_l / car_status.encoder_ppr * 2.0 * PI * wheel_radius;
}

geometry_msgs::Pose2D StatusPublisher::get_CarPos2D()
{
  return CarPos2D;
}

geometry_msgs::Twist StatusPublisher::get_CarTwist()
{
  return CarTwist;
}

std_msgs::Float64 StatusPublisher::get_power()
{
  return CarPower;
}

nav_msgs::Odometry StatusPublisher::get_odom()
{
  return CarOdom;
}
int StatusPublisher::get_status()
{
  return car_status.status;
}

void StatusPublisher::get_current_counters(float & sr, float & sl, float & theta, float & t, float & vtheta)
{
  boost::mutex::scoped_lock lock(mMutex_counters);
  sr = msr;
  sl = msl;
  theta = mtheta;
  t = mt;
  vtheta = mvtheta;
}

void StatusPublisher::update_current_counters()
{
  boost::mutex::scoped_lock lock(mMutex_counters);
  msr += 1.0f*car_status.encoder_delta_r / car_status.encoder_ppr *2*PI;
  msl += 1.0f*car_status.encoder_delta_l / car_status.encoder_ppr *2*PI;
  mtheta = car_status.theta* PI / 180.0f;
  mvtheta = -car_status.IMU[5]* PI / 180.0f;
  mt = car_status.time_stamp*1.0f/500.0f;
}

} //namespace xqserial_server
