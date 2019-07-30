#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>
#include <deque>

#define DISABLE 0
#define ENABLE 1

namespace xqserial_server
{
typedef sensor_msgs::PointCloud2 PointCloud;

StatusPublisher::StatusPublisher()
{
    mbUpdated_car=false;
    mbUpdated_imu=false;
    wheel_separation=0.33;
    wheel_radius=0.07;
    power_scale_ =1.0;

    CarPos2D.x=0.0;
    CarPos2D.y=0.0;
    CarPos2D.theta=0.0;

    CarTwist.linear.x=0.0;
    CarTwist.linear.y=0.0;
    CarTwist.linear.z=0.0;
    CarTwist.angular.x=0.0;
    CarTwist.angular.y=0.0;
    CarTwist.angular.z=0.0;

    CarPower.data = 0.0;

    int i=0;
    int * status;
    status=(int *)&car_status;
    for(i=0;i<30;i++)
    {
        status[i]=0;
    }

    car_status.encoder_ppr=45720;
    car_status.status_imu = -1;
    car_status.driver_status = 0;
    car_status.status = -1;

    for(i=0;i<4;i++)
    {
      car_status.sonar_distance[i]=4.2;
    }
   mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D",1,true);
   mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag",1,true);
   mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist",1,true);
   mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
   mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);
   mIMUPub = mNH.advertise<sensor_msgs::Imu>("xqserial_server/IMU", 1, true);

   mSonar1Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar1", 1, true);
   mSonar2Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar2", 1, true);
   mSonar3Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar3", 1, true);
   mSonar4Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar4", 1, true);

   CarSonar1.header.frame_id = "sonar1";
   CarSonar1.radiation_type = 0;
   CarSonar1.field_of_view = 0.7;
   CarSonar1.min_range = 0.2;
   CarSonar1.max_range = 4.2;

   CarSonar2.header.frame_id = "sonar2";
   CarSonar2.radiation_type = 0;
   CarSonar2.field_of_view = 0.7;
   CarSonar2.min_range = 0.2;
   CarSonar2.max_range = 4.2;

   CarSonar3.header.frame_id = "sonar3";
   CarSonar3.radiation_type = 0;
   CarSonar3.field_of_view = 0.7;
   CarSonar3.min_range = 0.2;
   CarSonar3.max_range = 4.2;

   CarSonar4.header.frame_id = "sonar4";
   CarSonar4.radiation_type = 0;
   CarSonar4.field_of_view = 0.7;
   CarSonar4.min_range = 0.2;
   CarSonar4.max_range = 4.2;
  /* static tf::TransformBroadcaster br;
   tf::Quaternion q;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.13) );//摄像头距离地面高度13cm
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
   */

   debug_flag=false;

   forward_flag_ = true;
   rot_flag_ = true;

   rot_dist_ = -0.21;
   tran_dist_ = -0.3;
   distances_[0] = 4.2;
   distances_[1] = 4.2;
   base_time_ = ros::Time::now().toSec();
}

StatusPublisher::StatusPublisher(double separation,double radius,bool debugFlag,double power_scale)
{
    new (this)StatusPublisher();
    wheel_separation=separation;
    wheel_radius=radius;
    debug_flag=debugFlag;
    power_scale_ = power_scale;
}

void StatusPublisher::Update_car(const char data[], unsigned int len)
{
    ROS_DEBUG("receive one package! %s , len %d",data,len);
    return;
}

void StatusPublisher::Update_imu(const char data[], unsigned int len)
{
    int i=0,j=0;
    int * receive_byte;
    static unsigned char last_str[2]={0x00,0x00};
    static unsigned char new_packed_ctr=DISABLE;//ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len=0;//包的理论长度
    static int new_packed_len=0;//包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str=0x00;
    const int cmd_string_max_size=512;
    receive_byte=(int *)&car_status;

    for(i=0;i<len;i++)
    {
      //ROS_ERROR("current2 %x",data[i]);
        current_str=data[i];
        //判断是否有新包头
        if(last_str[0]==205&&last_str[1]==235&&current_str==215) //包头 205 235 215
        {
            //std::cout<<"runup1 "<<std::endl;
            new_packed_ctr=ENABLE;
            new_packed_ok_len=0;
            new_packed_len=new_packed_ok_len;
            last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
            last_str[1]=current_str;
            continue;
        }
        last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
        last_str[1]=current_str;
        if (new_packed_ctr==ENABLE)
        {

            //获取包长度
            new_packed_ok_len=current_str;
            if(new_packed_ok_len>cmd_string_max_size) new_packed_ok_len=cmd_string_max_size; //包内容最大长度有限制
            new_packed_ctr=DISABLE;
            //std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        else
        {
            //判断包当前大小
            if(new_packed_ok_len<=new_packed_len)
            {
                //std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，后续内容无效
                continue;
            }
            else
            {
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len-1]=current_str;
                if(new_packed_ok_len==new_packed_len&&new_packed_ok_len>0)
                {
                    //std::cout<<"runup4 "<<std::endl;
                    boost::mutex::scoped_lock lock(mMutex_imu);
                    //当前包已经处理完成，开始处理
                    if(new_packed_ok_len==130)
                    {
                      for(j=0;j<26;j++)
                      {
                          memcpy(&receive_byte[j],&cmd_string_buf[5*j],4);
                      }
                      mbUpdated_imu=true;
                    }


                    if(mbUpdated_imu)
                    {
                      for(j=0;j<7;j++)
                      {
                          if(cmd_string_buf[5*j+4]!=32)
                          {
                            mbUpdated_imu=false;
                            car_status.encoder_ppr = 45720;
                            break;
                          }
                      }
                    }
                    if (mbUpdated_imu)
                    {
                      base_time_ = ros::Time::now().toSec();
                    }
                    new_packed_ok_len=0;
                    new_packed_len=0;
                }
            }

        }

    }

    return;
}

void StatusPublisher::Refresh()
{
  boost::mutex::scoped_lock lock(mMutex_imu);
  static double theta_last = 0.0;
  static unsigned int ii = 0;
  static bool theta_updateflag = false;
  static bool theta_update_first = true;
  static double delta_theta_last = 0;
  ii++;
  //std::cout<<"runR"<< mbUpdated<<std::endl;
  if (mbUpdated_imu)
  {
    // Time
    ros::Time current_time;

    if (car_status.status != 0)
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

    if (std::isnan(delta_car)||delta_car > 0.10|| delta_car < -0.10)
    {
      // std::cout<<"get you!"<<std::endl;
      delta_car = 0;
    }

    delta_x = delta_car * cos(CarPos2D.theta * PI / 180.0f);
    delta_y = delta_car * sin(CarPos2D.theta * PI / 180.0f);

    if(theta_update_first && theta_updateflag)
    {
      theta_last=car_status.theta;
      theta_update_first=false;
    }
    delta_theta = car_status.theta - theta_last;
    if (delta_theta > 270)
      delta_theta -= 360;
    if (delta_theta < -270)
      delta_theta += 360;

    if ((!theta_updateflag) ||std::isnan(delta_theta)|| delta_theta > 20 || delta_theta < -20)
    {
      delta_theta = 0;
    }
    else
    {
      theta_last = car_status.theta;
    }

    delta_theta_last = delta_theta;

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
    if(car_status.status_imu == 1 )
    {
      //反映红外状态
      car_status.status = 0;
    }
    else
    {
      //反映imu错误
      car_status.status = -1;
    }

    if(car_status.status == 0)
    {
      //反映驱动板错误
      boost::mutex::scoped_lock lock(mMutex_car);
      car_status.status = car_status.driver_status;
    }

    flag.data = car_status.status;

    mStatusFlagPub.publish(flag);

  	//Twist
  	static float v_sums[8] = { 0,0,0,0,0,0,0,0 }, v_sum = 0, v_set = 0;
  	static float theta_sums[8] = { 0,0,0,0,0,0,0,0 }, theta_sum = 0, theta_set = 0;
  	static int v_sum_index = 0;
  	static int theta_sum_index = 0;
  	{
  		//平滑
  		v_sums[v_sum_index] = delta_car*50.0f;
      v_sum = 0;
      for(int j =0; j<8;j++)
      {
        v_sum += v_sums[j];
      }

  		CarTwist.linear.x = v_sum / 8.0f;
  		v_sum_index++;
  		if (v_sum_index>7) v_sum_index = 0;

  		double angle_speed;
  		if (car_status.upwoard == 0)
  		{
  			angle_speed = -car_status.IMU[5];
  		}
  		else
  		{
  			angle_speed = car_status.IMU[5];
  		}
  		static float angle_speed_last = 0;
  		if (std::isnan(angle_speed) || std::fabs(angle_speed)>500)
  		{
  			angle_speed = angle_speed_last;
  		}
  		else
  		{
  			angle_speed_last = angle_speed;
  		}
  		theta_sums[theta_sum_index] = angle_speed * PI / 180.0f;
      theta_sum = 0;
      for(int j =0; j<8;j++)
      {
        theta_sum += theta_sums[j];
      }
  		CarTwist.angular.z = theta_sum / 8.0f;//angle_speed*PI / 180.0f;
  		theta_sum_index++;
  		if (theta_sum_index>7) theta_sum_index = 0;
  		//std::cout<<" " << angle_speed * PI /180.0f<<std::endl;
  		//ROS_ERROR("%d %f,%f,%f,%f,%f,%f,%f,%f,%f",theta_sum_index,theta_sum,theta_sums[0],theta_sums[1],theta_sums[2],theta_sums[3],theta_sums[4],theta_sums[5],theta_sums[6],theta_sums[7]);
  		// CarTwist.linear.x=delta_car*50.0f;
  		// CarTwist.angular.z=angle_speed * PI /180.0f;
  	}
  	mTwistPub.publish(CarTwist);

    CarPower.data = car_status.power_imu*power_scale_;
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

  	if (car_status.upwoard == 0)
  	{
  		CarIMU.angular_velocity.x = car_status.IMU[4] * PI / 180.0f;
  		CarIMU.angular_velocity.y = car_status.IMU[3] * PI / 180.0f;
  		CarIMU.angular_velocity.z = -car_status.IMU[5] * PI / 180.0f;

  		CarIMU.linear_acceleration.x = car_status.IMU[1];
  		CarIMU.linear_acceleration.y = car_status.IMU[0];
  		CarIMU.linear_acceleration.z = -car_status.IMU[2];
  	}
  	else
  	{
  		CarIMU.angular_velocity.x = car_status.IMU[4] * PI / 180.0f;
  		CarIMU.angular_velocity.y = -car_status.IMU[3] * PI / 180.0f;
  		CarIMU.angular_velocity.z = car_status.IMU[5] * PI / 180.0f;

  		CarIMU.linear_acceleration.x = car_status.IMU[1];
  		CarIMU.linear_acceleration.y = -car_status.IMU[0];
  		CarIMU.linear_acceleration.z = car_status.IMU[2];
  	}
    mIMUPub.publish(CarIMU);

    //超声波测距
    //发布超声波topic
    rot_flag_ = true;
    if(car_status.sonar_distance[0]>0.1)
    {
      if(car_status.sonar_distance[0]>4.0||car_status.sonar_distance[0]<0.2) car_status.sonar_distance[0]=4.0;
      CarSonar1.header.stamp = current_time.fromSec(base_time_);
      CarSonar1.range = car_status.sonar_distance[0];
      mSonar1Pub.publish(CarSonar1);
      if(car_status.sonar_distance[0]<rot_dist_ && rot_flag_) rot_flag_ = false;
    }
    else
    {
      car_status.sonar_distance[0]=4.0;
    }

    forward_flag_ = true;
    if(car_status.sonar_distance[1]>0.1)
    {
      if(car_status.sonar_distance[1]>4.0||car_status.sonar_distance[1]<0.2) car_status.sonar_distance[1]=4.0;
      CarSonar2.header.stamp = current_time.fromSec(base_time_);
      CarSonar2.range = car_status.sonar_distance[1];
      mSonar2Pub.publish(CarSonar2);
      if(car_status.sonar_distance[1]<tran_dist_ && forward_flag_) forward_flag_ = false;
    }
    else
    {
      car_status.sonar_distance[1]=4.0;
    }

    if(car_status.sonar_distance[2]>0.1)
    {
      if(car_status.sonar_distance[2]>4.0||car_status.sonar_distance[2]<0.2) car_status.sonar_distance[2]=4.0;
      CarSonar3.header.stamp = current_time.fromSec(base_time_);
      CarSonar3.range = car_status.sonar_distance[2];
      mSonar3Pub.publish(CarSonar3);
      if(car_status.sonar_distance[2]<rot_dist_ && rot_flag_) rot_flag_ = false;
    }
    else
    {
      car_status.sonar_distance[2]=4.0;
    }

    if(car_status.sonar_distance[3]>0.1)
    {
      if(car_status.sonar_distance[3]>4.0||car_status.sonar_distance[3]<0.2) car_status.sonar_distance[3]=4.0;
      CarSonar4.header.stamp = current_time.fromSec(base_time_);
      CarSonar4.range = car_status.sonar_distance[3];
      mSonar4Pub.publish(CarSonar4);
      if(car_status.sonar_distance[3]<tran_dist_ && forward_flag_) forward_flag_ = false;
    }
    else
    {
      car_status.sonar_distance[3]=4.0;
    }

    distances_[0] = car_status.sonar_distance[1];
    distances_[1] = car_status.sonar_distance[3];

    // pub transform

    static tf::TransformBroadcaster br;
    tf::Quaternion q;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(CarPos2D.x, CarPos2D.y, 0.0));
    q.setRPY(0, 0, CarPos2D.theta / 180 * PI);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, current_time.fromSec(base_time_), "odom", "base_footprint"));

    mbUpdated_imu = false;
  }
}

double StatusPublisher::get_wheel_separation(){
    return wheel_separation;
}

double StatusPublisher::get_wheel_radius(){
    return wheel_radius;
}

int StatusPublisher::get_wheel_ppr(){
    return car_status.encoder_ppr;
}

void StatusPublisher::get_wheel_speed(double speed[2]){
    //右一左二
    speed[0]=car_status.encoder_delta_r*50.0/car_status.encoder_ppr*2.0*PI*wheel_radius;
    speed[1]=car_status.encoder_delta_l*50.0/car_status.encoder_ppr*2.0*PI*wheel_radius;
}

geometry_msgs::Pose2D StatusPublisher::get_CarPos2D(){
    return CarPos2D;
}

geometry_msgs::Twist StatusPublisher::get_CarTwist(){
    return CarTwist;
}

std_msgs::Float64 StatusPublisher::get_power(){
  return CarPower;
}

nav_msgs::Odometry StatusPublisher::get_odom(){
  return CarOdom;
}

int StatusPublisher::get_status(){
  return car_status.status;
}

void StatusPublisher::get_canmove_flag(bool &forward_flag,bool &rot_flag)
{
  forward_flag = forward_flag_;
  rot_flag = rot_flag_;
}

float StatusPublisher::get_ultrasonic_min_distance()
{
  return std::min(distances_[0],distances_[1]);
}

} //namespace xqserial_server
