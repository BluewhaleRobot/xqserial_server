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
    mbUpdated_car=false;
    mbUpdated_imu=false;
    wheel_separation=0.42;
    wheel_radius=0.086;

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
    for(i=0;i<35;i++)
    {
        status[i]=0;
    }
    car_status.encoder_ppr=5600;

   mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D",1,true);
   mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag",1,true);
   mDriverFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/DriverFlag",1,true);

   mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist",1,true);
   mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
   mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);
   pub_barpoint_cloud_ = mNH.advertise<PointCloud>("kinect/barpoints", 1, true);
   pub_clearpoint_cloud_ = mNH.advertise<PointCloud>("kinect/clearpoints", 1, true);

   mSonar1Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar1", 1, true);
   mSonar2Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar2", 1, true);
   mSonar3Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar3", 1, true);
   mSonar4Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar4", 1, true);

   mIMUPub = mNH.advertise<sensor_msgs::Imu>("xqserial_server/IMU", 1, true);

   CarSonar1.header.frame_id = "sonar1";
   CarSonar1.radiation_type = 0;
   CarSonar1.field_of_view = 0.7;
   CarSonar1.min_range = 0.19;
   CarSonar1.max_range = 4.2;

   CarSonar2.header.frame_id = "sonar2";
   CarSonar2.radiation_type = 0;
   CarSonar2.field_of_view = 0.7;
   CarSonar2.min_range = 0.19;
   CarSonar2.max_range = 4.2;

   CarSonar3.header.frame_id = "sonar3";
   CarSonar3.radiation_type = 0;
   CarSonar3.field_of_view = 0.7;
   CarSonar3.min_range = 0.19;
   CarSonar3.max_range = 4.2;

   CarSonar4.header.frame_id = "sonar4";
   CarSonar4.radiation_type = 0;
   CarSonar4.field_of_view = 0.7;
   CarSonar4.min_range = 0.19;
   CarSonar4.max_range = 4.2;

   power_scale_ = 1.0;
   forward_flag_ = true;
   rot_flag_ = true;

   rot_dist_ = -0.21;
   tran_dist_ = -0.3;

   yaw_index=0;
   yaw_sum=0;
   yaw_omega=0;
   for(int i=0;i<100;i++)
   {
     yaw_deltas[i]=0.0;
   }
   encoder_r_last = 0;
   encoder_l_last = 0;

   distances_[0] = 4.0;
   distances_[1] = 4.0;
   distances_[2] = 4.0;
   distances_[3] = 4.0;
}

StatusPublisher::StatusPublisher(double separation,double radius,double power_scale)
{
    new (this)StatusPublisher();
    wheel_separation=separation;
    wheel_radius=radius;
    power_scale_ = power_scale;
    last_sonartime_ = ros::WallTime::now();
    min_sonardist_ = 0.0;
}

void StatusPublisher::Update_car(const char data[], unsigned int len)
{
    int i=0,j=0;
    int * receive_byte;
    static std::deque<unsigned char>  cmd_string_buf={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    unsigned char current_str=0x00;

    for(i=0;i<len;i++)
    {
        current_str=data[i];
        cmd_string_buf.pop_front();
        cmd_string_buf.push_back(current_str);
        //std::cout<< std::hex  << (int)current_str <<std::endl;
        //ROS_ERROR("current %d %d",data[i],i);
        if(cmd_string_buf[0]!=0x01 ) //校验站号
        {
          continue;
        }

        if(cmd_string_buf[1]!=0x43 ) //校验功能码
        {
          continue;
        }

        //校验crc16
        uint8_t crc_hl[2];
        unsigned char cmd_string_buf_copy[10] = {cmd_string_buf[0],cmd_string_buf[1],cmd_string_buf[2],cmd_string_buf[3],cmd_string_buf[4],cmd_string_buf[5],cmd_string_buf[6],cmd_string_buf[7],cmd_string_buf[8],cmd_string_buf[9]};
        xqserial_server::CRC16CheckSum(cmd_string_buf_copy, 10, crc_hl);
        //ROS_ERROR("crc %x %x , in %x %x",crc_hl[0],crc_hl[1],cmd_string_buf[7],cmd_string_buf[8]);
        if(cmd_string_buf[10]!=crc_hl[0] || cmd_string_buf[11]!=crc_hl[1] )
        {
          continue;
        }
        //ROS_ERROR("receive one package!");
        //有效包，开始提取数据
        {
          //右2 左1
          boost::mutex::scoped_lock lock(mMutex_car);
          unsigned short int data1_address = cmd_string_buf[2]<<8|cmd_string_buf[3];
          unsigned short int data2_address = cmd_string_buf[4]<<8|cmd_string_buf[5];
          short int data1 = cmd_string_buf[6]<<8|cmd_string_buf[7];
          short int data2 = cmd_string_buf[8]<<8|cmd_string_buf[9];

          switch (data1_address) {
            //右轮电机
            case 0x2100:
              //使能状态
              car_status.driver_enable1 = data1;
              break;
            case 0x5012:
              //错误状态
              car_status.driver_error1 = data1;
              break;
            case 0x5004:
              //编码器位置
              car_status.encoder_l_current = data1;
              break;
          }

          switch (data2_address) {
            //右轮电机
            case 0x3100:
              //使能状态
              car_status.driver_enable2 = data2;
              break;
            case 0x5112:
              //错误状态
              car_status.driver_error2 = data2;
              break;
            case 0x5104:
              //编码器位置
              car_status.encoder_r_current = data2;
              break;
          }

          car_status.driver_error = car_status.driver_error1 + car_status.driver_error2;
          mbUpdated_car = true;
        }
    }

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
                            break;
                          }
                      }
                    }
                    //ROS_ERROR("status %d",car_status.status);
                    // if (mbUpdated_imu)
                    // {
                    //   base_time_ = ros::Time::now().toSec();
                    // }
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
    static unsigned int ii=0;
    ii++;
    int delta_encoder_r = 0;
    int delta_encoder_l = 0;
    //ROS_ERROR("car imu %d %d",mbUpdated_imu, mbUpdated_car);
    //先处理驱动器
    {
      boost::mutex::scoped_lock lock(mMutex_car);
      if(ii%5 == 0)
      {
        //发布驱动器状态
        std_msgs::Int32 driver_flag;
        driver_flag.data = car_status.driver_error1 << 8 + car_status.driver_error2;
        mDriverFlagPub.publish(driver_flag);
      }

      delta_encoder_r = car_status.encoder_r_current - encoder_r_last;
      encoder_r_last = car_status.encoder_r_current;
      delta_encoder_l = car_status.encoder_l_current - encoder_l_last;
      encoder_l_last = car_status.encoder_l_current;

      if(delta_encoder_r>2000)
      {
        delta_encoder_r = delta_encoder_r - car_status.encoder_ppr;
      }
      if(delta_encoder_r<-2000)
      {
        delta_encoder_r = delta_encoder_r + car_status.encoder_ppr;
      }

      if(delta_encoder_l>2000)
      {
        delta_encoder_l = delta_encoder_l - car_status.encoder_ppr;
      }
      if(delta_encoder_l<-2000)
      {
        delta_encoder_l = delta_encoder_l + car_status.encoder_ppr;
      }

      delta_encoder_l = -delta_encoder_l; //左侧电机反向

      //ROS_ERROR("delta_encoder_r delta_encoder_l %d %d, car_status.encoder_r_current car_status.encoder_l_current %d %d",delta_encoder_r, delta_encoder_l,car_status.encoder_r_current,car_status.encoder_l_current);
      mbUpdated_car = false;
    }

    //处理传感器数据
    static float theta_last = 0.0;
    float delta_theta = 0;
    static int update_nums=0;
    static bool yaw_omega_ready = false;
    {
      boost::mutex::scoped_lock lock(mMutex_imu);
      if(mbUpdated_imu)
      {
        //处理imu
        ros::Time current_time = ros::Time::now();

        if(car_status.status <= 0)
        {
          //复位状态
          theta_last = car_status.theta_imu;
          yaw_omega = 0;
          for(int i=0;i<100;i++)
          {
            yaw_deltas[i]=0.0;
          }
          yaw_index = 0;
          yaw_omega_ready = false;
          delta_theta = 0;
          delta_encoder_r = 0;
          delta_encoder_l = 0;
        }
        else
        {
          delta_theta = car_status.theta_imu - theta_last - yaw_omega;
          //ROS_ERROR("delta %f , theta_raw %f, yaw_omega %f", delta_theta, car_status.theta_imu,yaw_omega);
          if( std::fabs(car_status.theta_imu - theta_last)<0.01 && std::fabs(delta_encoder_r) < 2 && std::fabs(delta_encoder_l) < 2)
          {
            if(update_nums>50)
            {
              delta_theta = 0;
              //更新yaw_omega
              yaw_sum -= yaw_deltas[yaw_index];
              yaw_deltas[yaw_index] = car_status.theta_imu - theta_last;
              yaw_sum += yaw_deltas[yaw_index];

              yaw_index++;
              if(yaw_index>99)
              {
               yaw_index = 0;
               yaw_omega_ready = true;
              }

              if(yaw_omega_ready) yaw_omega = yaw_sum/100.0;

              //ROS_ERROR("delta_yaw %f ,  yaw_omega %f", car_status.theta_imu - theta_last,yaw_omega);
            }
            else
            {
              update_nums++;
            }
          }
          else
          {
            if(update_nums > 0) update_nums--;
          }
          theta_last = car_status.theta_imu;
        }

        if(delta_theta > 270 ) delta_theta -= 360;
        if(delta_theta < -270 ) delta_theta += 360;

        if( std::isnan(delta_theta) ||delta_theta>20||delta_theta<-20)
        {
          delta_theta = 0;
        }
        car_status.theta += delta_theta;
        if( car_status.theta > 360) car_status.theta -= 360;
        if( car_status.theta < 0 ) car_status.theta += 360;

        //发布imu twist odom
        float delta_car,delta_x,delta_y,var_len,var_angle;

        var_len = (50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius);
        var_angle = (0.01f/180.0f*PI)*(0.01f/180.0f*PI);

        delta_car = (delta_encoder_r + delta_encoder_l)/2.0f*1.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius;
        if(std::isnan(delta_theta) ||delta_car>0.05||delta_car<-0.05)
        {
          delta_car = 0;
        }

        delta_x=delta_car*cos(CarPos2D.theta* PI / 180.0f);
        delta_y=delta_car*sin(CarPos2D.theta* PI / 180.0f);

        CarPos2D.x+=delta_x;
        CarPos2D.y+=delta_y;
        CarPos2D.theta+=delta_theta;

        if(CarPos2D.theta>360.0) CarPos2D.theta-=360;
        if(CarPos2D.theta<0.0) CarPos2D.theta+=360;

        mPose2DPub.publish(CarPos2D);

        //flag
        std_msgs::Int32 flag;
        flag.data=car_status.status;
        //底层障碍物信息
        if((car_status.hbz_status & 0x01)==0 && car_status.status > 0)
        {
          //急停被按下了
          flag.data=2;
        }
        mStatusFlagPub.publish(flag);

        //Twist
        static float v_sums[8]={0,0,0,0,0,0,0,0},v_sum=0,v_set=0;
        static float theta_sums[8]={0,0,0,0,0,0,0,0},theta_sum=0,theta_set=0;
        static int v_sum_index=0;
        static int theta_sum_index=0;
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
        }

        mTwistPub.publish(CarTwist);

        CarPower.data = car_status.power*power_scale_;
        mPowerPub.publish(CarPower);

        CarOdom.header.stamp = current_time;
        CarOdom.header.frame_id = "odom";
        CarOdom.pose.pose.position.x = CarPos2D.x;
        CarOdom.pose.pose.position.y = CarPos2D.y;
        CarOdom.pose.pose.position.z = 0.0f;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(CarPos2D.theta/180.0f*PI);
        CarOdom.pose.pose.orientation = odom_quat;
        CarOdom.pose.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                                                              (0) (var_len)  (0)  (0)  (0)  (0)
                                                              (0)   (0)  (999) (0)  (0)  (0)
                                                              (0)   (0)   (0) (999) (0)  (0)
                                                              (0)   (0)   (0)  (0) (999) (0)
                                                              (0)   (0)   (0)  (0)  (0)  (var_angle) ;
        CarOdom.child_frame_id = "base_footprint";
        CarOdom.twist.twist.linear.x = CarTwist.linear.x;// * cos(CarPos2D.theta* PI / 180.0f);
        CarOdom.twist.twist.linear.y = CarTwist.linear.y;// * sin(CarPos2D.theta* PI / 180.0f);
        CarOdom.twist.twist.angular.z = CarTwist.angular.z;
        CarOdom.twist.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                                                              (0) (var_len)  (0)  (0)  (0)  (0)
                                                              (0)   (0)  (999) (0)  (0)  (0)
                                                              (0)   (0)   (0) (999) (0)  (0)
                                                              (0)   (0)   (0)  (0) (999) (0)
                                                              (0)   (0)   (0)  (0)  (0)  (var_angle) ;
        mOdomPub.publish(CarOdom);

        // pub transform

        static tf::TransformBroadcaster br;
        tf::Quaternion q;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(CarPos2D.x, CarPos2D.y, 0.0) );
        q.setRPY(0, 0, CarPos2D.theta/180*PI);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));


        //publish IMU
        tf::Quaternion q_imu;
        q_imu.setRPY(0, 0, car_status.theta/180.0*PI);
        CarIMU.header.stamp = current_time;
        CarIMU.header.frame_id = "imu";
        CarIMU.orientation.x=q_imu.x();
        CarIMU.orientation.y=q_imu.y();
        CarIMU.orientation.z=q_imu.z();
        CarIMU.orientation.w=q_imu.w();
        if(car_status.upwoard == 0)
        {
          CarIMU.angular_velocity.x=car_status.IMU[4]* PI /180.0f;
          CarIMU.angular_velocity.y=car_status.IMU[3]* PI /180.0f;
          CarIMU.angular_velocity.z=-car_status.IMU[5]* PI /180.0f;

          CarIMU.linear_acceleration.x=car_status.IMU[1];
          CarIMU.linear_acceleration.y=car_status.IMU[0];
          CarIMU.linear_acceleration.z=-car_status.IMU[2];
        }
        else
        {
          CarIMU.angular_velocity.x=-car_status.IMU[3]* PI /180.0f;
          CarIMU.angular_velocity.y=-car_status.IMU[4]* PI /180.0f;
          CarIMU.angular_velocity.z=car_status.IMU[5]* PI /180.0f;

          CarIMU.linear_acceleration.x=-car_status.IMU[0];
          CarIMU.linear_acceleration.y=-car_status.IMU[1];
          CarIMU.linear_acceleration.z=car_status.IMU[2];
        }

        mIMUPub.publish(CarIMU);

        //超声波测距
        static float distance1_sums[8]={4.0,4.0,4.0,4.0,4.0,4.0,4.0,4.0},distance1_sum=8.0;
        static float distance2_sums[8]={4.0,4.0,4.0,4.0,4.0,4.0,4.0,4.0},distance2_sum=8.0;
        static float distance3_sums[8]={4.0,4.0,4.0,4.0,4.0,4.0,4.0,4.0},distance3_sum=8.0;
        static float distance4_sums[8]={4.0,4.0,4.0,4.0,4.0,4.0,4.0,4.0},distance4_sum=8.0;
        static int distance_sum_index=0,distance_sum_i=0;

        if(distance_sum_i%10==0)
        {
          //平滑
          forward_flag_ = true;
          // distance1_sum -=distance1_sums[distance_sum_index];
          // distance1_sums[distance_sum_index] = car_status.sonar_distance[0];
          // distance1_sum +=distance1_sums[distance_sum_index];
          //
          // distance2_sum -=distance2_sums[distance_sum_index];
          // distance2_sums[distance_sum_index] = car_status.sonar_distance[1];
          // distance2_sum +=distance2_sums[distance_sum_index];
          //
          // distance3_sum -=distance3_sums[distance_sum_index];
          // distance3_sums[distance_sum_index] = car_status.sonar_distance[2];
          // distance3_sum +=distance3_sums[distance_sum_index];
          //
          // distance4_sum -=distance4_sums[distance_sum_index];
          // distance4_sums[distance_sum_index] = car_status.sonar_distance[3];
          // distance4_sum +=distance4_sums[distance_sum_index];
          //
          // distance_sum_index++;
          // if(distance_sum_index>1) distance_sum_index = 0;

          // distances_[0] = distance1_sum/2.0f;
          // distances_[1] = distance2_sum/2.0f;
          // distances_[2] = distance3_sum/2.0f;
          // distances_[3] = distance4_sum/2.0f;
          distances_[0] = car_status.sonar_distance[0];
          distances_[1] = car_status.sonar_distance[1];
          distances_[2] = car_status.sonar_distance[2];
          distances_[3] = car_status.sonar_distance[3];

          //std::cout<<" " << car_status.distance1 << " " << car_status.distance2<<std::endl;
           //发布超声波topic
          if(distances_[0]>0.1)
          {
           CarSonar1.header.stamp = current_time;
           CarSonar1.range = distances_[0];
           mSonar1Pub.publish(CarSonar1);
           if(distances_[0]<tran_dist_ && forward_flag_)
           {
             forward_flag_ = false;
             //ROS_ERROR("sonar1 %f",distances_[0]);
           }
          }

          if(distances_[1]>0.1)
          {
           CarSonar2.header.stamp = current_time;
           CarSonar2.range = distances_[1];
           mSonar2Pub.publish(CarSonar2);
           if(distances_[1]<tran_dist_ && forward_flag_)
           {
             forward_flag_ = false;
             //ROS_ERROR("sonar2 %f",distances_[1]);
           }
          }

          if(distances_[2]>0.1)
          {
           CarSonar3.header.stamp = current_time;
           CarSonar3.range = distances_[2];
           mSonar3Pub.publish(CarSonar3);
           if(distances_[2]<tran_dist_ && forward_flag_)
           {
             forward_flag_ = false;
             //ROS_ERROR("sonar3 %f",distances_[2]);
           }
          }

          if(distances_[3]>0.1)
          {
           CarSonar4.header.stamp = current_time;
           CarSonar4.range = distances_[3];
           mSonar4Pub.publish(CarSonar4);
           if(distances_[3]<tran_dist_ && forward_flag_)
           {
             forward_flag_ = false;
             //ROS_ERROR("sonar4 %f",distances_[3]);
           }
          }
           //ROS_ERROR("forward_flag_ %d %f %f",(int)forward_flag_,distances_[1],car_status.sonar_distance[1]);
        }
        else
        {
          //匀减速模型
          const float dt = 0.02;
          distances_[0] = distances_[0] - CarTwist.linear.x*dt;
          distances_[1] = distances_[1] - CarTwist.linear.x*dt;
          distances_[2] = distances_[2] - CarTwist.linear.x*dt;
          distances_[3] = distances_[3] - CarTwist.linear.x*dt;
        }
        distance_sum_i++;

        int barArea_nums=0;
        int clearArea_nums=0;
        static int hbz1_num=0,hbz2_num=0;

        if(distances_[1]<tran_dist_ || distances_[3]<tran_dist_)
        {
          hbz1_num++;
          if(hbz1_num>10) barArea_nums += 5;
          if(hbz1_num>15) hbz1_num = 15;
        }else{
          hbz1_num--;
          if(hbz1_num<0) hbz1_num = 0;
          if(hbz1_num==0) clearArea_nums += 10;
        }

        if(distances_[0]<tran_dist_ || distances_[2]<tran_dist_)
        {
          hbz2_num++;
          if(hbz2_num>10) barArea_nums += 5;
          if(hbz2_num>15) hbz2_num = 15;
        }else{
          hbz2_num--;
          if(hbz2_num<0) hbz2_num=0;
          if(hbz2_num==0) clearArea_nums+=10;
        }

        if(barArea_nums>0)
        {
          //发布雷区
          PointCloud::Ptr barcloud_msg(new PointCloud);
          barcloud_msg->header.stamp = current_time;
          barcloud_msg->height = 1;
          barcloud_msg->width  = barArea_nums;
          barcloud_msg->is_dense = true;
          barcloud_msg->is_bigendian = false;
          barcloud_msg->header.frame_id="kinect_link_new";
          sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
          pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
          sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
          sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
          sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
          if((distances_[1]<tran_dist_ || distances_[3]<tran_dist_)&&hbz1_num>10)
          {
            for(int k=0;k<5;k++,++bariter_x, ++bariter_y,++bariter_z)
            {
              *bariter_x=0.1+0.3;
              *bariter_y=k*0.04+0.15;
              *bariter_z=0.15;
            }
          }

          if((distances_[0]<tran_dist_ || distances_[2]<tran_dist_)&&hbz2_num>10)
          {
            for(int k=0;k<5;k++,++bariter_x, ++bariter_y,++bariter_z)
            {
              *bariter_x=0.1+0.3;
              *bariter_y=-k*0.04-0.15;
              *bariter_z=0.15;
            }
          }
          if(ii%1==0)
          {
            pub_barpoint_cloud_.publish(barcloud_msg);
          }
        }
        if(clearArea_nums>0)
        {
          //清除雷区
          PointCloud::Ptr clearcloud_msg(new PointCloud);
          clearcloud_msg->header.stamp = current_time;
          clearcloud_msg->height = 1;
          clearcloud_msg->width  = clearArea_nums;
          clearcloud_msg->is_dense = true;
          clearcloud_msg->is_bigendian = false;
          clearcloud_msg->header.frame_id="kinect_link_new";
          sensor_msgs::PointCloud2Modifier pcd_modifier1(*clearcloud_msg);
          pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
          sensor_msgs::PointCloud2Iterator<float> cleariter_x(*clearcloud_msg, "x");
          sensor_msgs::PointCloud2Iterator<float> cleariter_y(*clearcloud_msg, "y");
          sensor_msgs::PointCloud2Iterator<float> cleariter_z(*clearcloud_msg, "z");
          if(hbz2_num==0)
          {
            for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
            {
              *cleariter_x=0.1+0.3;
              *cleariter_y=k*0.04+0.15;
              *cleariter_z=0.0;
            }
            for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
            {
              *cleariter_x=0.05+0.3;
              *cleariter_y=k*0.04+0.15;
              *cleariter_z=0.0;
            }
          }
          if(hbz1_num==0)
          {
            for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
            {
              *cleariter_x=0.1+0.3;
              *cleariter_y=-k*0.04-0.15;
              *cleariter_z=0.0;
            }
            for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
            {
              *cleariter_x=0.05+0.3;
              *cleariter_y=-k*0.04-0.15;
              *cleariter_z=0.0;
            }
          }
          if(ii%1==0)
          {
            pub_clearpoint_cloud_.publish(clearcloud_msg);
          }
        }
      }
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
    speed[0]=car_status.encoder_delta_r*50/car_status.encoder_ppr*2.0*PI*wheel_radius;
    speed[1]=car_status.encoder_delta_l*50/car_status.encoder_ppr*2.0*PI*wheel_radius;
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

void StatusPublisher::get_distances(double distances[4])
{
  boost::mutex::scoped_lock lock(mMutex_imu);
  distances[0]=distances_[0];
  distances[1]=distances_[1];
  distances[2]=distances_[2];
  distances[3]=distances_[3];
}

void StatusPublisher::get_canmove_flag(bool &forward_flag,bool &rot_flag)
{
  forward_flag = forward_flag_;
  rot_flag = rot_flag_;
}

float StatusPublisher::get_ultrasonic_min_distance()
{
  boost::mutex::scoped_lock lock(mMutex_imu);
  return std::min(std::min(distances_[0],distances_[1]),std::min(distances_[2],distances_[3]));
}

} //namespace xqserial_server
