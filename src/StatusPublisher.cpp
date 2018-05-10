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
    for(i=0;i<39;i++)
    {
        status[i]=0;
    }
    car_status.encoder_ppr=4096*7.5;
    for(i=0;i<4;i++)
    {
      car_status.distance[i]=-0.2;
    }
   mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D",1,true);
   mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag",1,true);
   mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist",1,true);
   mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
   mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);
   pub_barpoint_cloud_ = mNH.advertise<PointCloud>("kinect/barpoints", 1, true);
   pub_clearpoint_cloud_ = mNH.advertise<PointCloud>("kinect/clearpoints", 1, true);
   mIMUPub = mNH.advertise<sensor_msgs::Imu>("xqserial_server/IMU", 1, true);
   mSonar1Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar1", 1, true);
   mSonar2Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar2", 1, true);
   mSonar3Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar3", 1, true);
   mSonar4Pub = mNH.advertise<sensor_msgs::Range>("xqserial_server/Sonar4", 1, true);
  /* static tf::TransformBroadcaster br;
   tf::Quaternion q;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.13) );//摄像头距离地面高度13cm
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
   */
   debug_flag=false;
   yaw_index=0;
   yaw_sum=0;
   yaw_omega=0;
   yaw_ready=false;
   for(int i=0;i<100;i++)
   {
     yaw_deltas[i]=0.0;
   }
}

StatusPublisher::StatusPublisher(double separation,double radius,bool debugFlag)
{
    new (this)StatusPublisher();
    wheel_separation=separation;
    wheel_radius=radius;
    debug_flag=debugFlag;
    CarSonar1.header.frame_id = "sonar1";
    CarSonar1.radiation_type = 0;
    CarSonar1.field_of_view = 0.7;
    CarSonar1.min_range = 0.19;
    CarSonar1.max_range = 4.1;

    CarSonar2.header.frame_id = "sonar2";
    CarSonar2.radiation_type = 0;
    CarSonar2.field_of_view = 0.7;
    CarSonar2.min_range = 0.19;
    CarSonar2.max_range = 4.1;

    CarSonar3.header.frame_id = "sonar3";
    CarSonar3.radiation_type = 0;
    CarSonar3.field_of_view = 0.7;
    CarSonar3.min_range = 0.19;
    CarSonar3.max_range = 4.1;

    CarSonar4.header.frame_id = "sonar4";
    CarSonar4.radiation_type = 0;
    CarSonar4.field_of_view = 0.7;
    CarSonar4.min_range = 0.19;
    CarSonar4.max_range = 4.1;
}

void StatusPublisher::Update_car(const char data[], unsigned int len)
{
    int i=0,j=0;
    int * receive_byte;
    static std::deque<unsigned char>  cmd_string_buf={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    static int8_t sum=0;
    unsigned char current_str=0x00;

    for(i=0;i<len;i++)
    {
        current_str=data[i];
        //std::cout<< std::hex  << (int)current_str <<std::endl;
        unsigned char sum_check = (unsigned char) (-sum);

        if((cmd_string_buf[0]!=0x01 && cmd_string_buf[0]!=0x02)||sum_check!=current_str) //校验包头和包和
        {
          sum -= cmd_string_buf[0];
          cmd_string_buf.pop_front();
          cmd_string_buf.push_back(current_str);
          sum += current_str;
          continue;
        }
        //校验res
        if(cmd_string_buf[1]<0x40||cmd_string_buf[1]>0x50)
        {
          sum -= cmd_string_buf[0];
          cmd_string_buf.pop_front();
          cmd_string_buf.push_back(current_str);
          sum += current_str;
          continue;
        }
        //有效包，开始提取数据
        {
          boost::mutex::scoped_lock lock(mMutex_car);
          unsigned int data_address = cmd_string_buf[3]<<16|cmd_string_buf[2]<<8|cmd_string_buf[4];
          unsigned char * data_byte;
          switch (data_address) {
            case 0x604100:
              //驱动器报警 16us,右1左2
              if(cmd_string_buf[0]==0x01)
              {
                car_status.driver_error=0;
                car_status.driver_error=cmd_string_buf[5];
              }
              if(cmd_string_buf[0]==0x02)
              {
                car_status.driver_error += cmd_string_buf[5];
              }
              break;
            case 0x606300:
              //位置 32s,右1左2
              if(cmd_string_buf[0]==0x01) data_byte = (unsigned char *)&car_status.encoder_r_current;
              if(cmd_string_buf[0]==0x02) data_byte = (unsigned char *)&car_status.encoder_l_current;
              data_byte[0]=cmd_string_buf[5];
              data_byte[1]=cmd_string_buf[6];
              data_byte[2]=cmd_string_buf[7];
              data_byte[3]=cmd_string_buf[8];
              if(cmd_string_buf[0]==0x02)
              {
                mbUpdated_car=true;
              }
              break;
            case 0x430000:
              //协同使能 8u
              data_byte = (unsigned char *)&car_status.mode_enable;
              data_byte[0]=cmd_string_buf[5];
              data_byte[1]=cmd_string_buf[6];
              data_byte[2]=cmd_string_buf[7];
              data_byte[3]=cmd_string_buf[8];
              break;
            case 0x430001:
              //协同模式 8s
              data_byte = (unsigned char *)&car_status.mode;
              data_byte[0]=cmd_string_buf[5];
              data_byte[1]=cmd_string_buf[6];
              data_byte[2]=cmd_string_buf[7];
              data_byte[3]=cmd_string_buf[8];
              break;
            case 0x430002:
              //协同控制字 8s
              data_byte = (unsigned char *)&car_status.mode_power_control;
              data_byte[0]=cmd_string_buf[5];
              data_byte[1]=cmd_string_buf[6];
              data_byte[2]=cmd_string_buf[7];
              data_byte[3]=cmd_string_buf[8];
              break;
            case 0x201002:
              //快速停止状态 16u
              data_byte = (unsigned char *)&car_status.faster_stop;
              data_byte[0]=cmd_string_buf[5];
              data_byte[1]=cmd_string_buf[6];
              data_byte[2]=cmd_string_buf[7];
              data_byte[3]=cmd_string_buf[8];
              break;
            case 0x430204:
              //协同运动速度 32s
              data_byte = (unsigned char *)&car_status.synergy_speed_set;
              data_byte[0]=cmd_string_buf[5];
              data_byte[1]=cmd_string_buf[6];
              data_byte[2]=cmd_string_buf[7];
              data_byte[3]=cmd_string_buf[8];
              break;
            case 0x430210:
              //协同运动半径 32s
              data_byte = (unsigned char *)&car_status.synergy_r_set;
              data_byte[0]=cmd_string_buf[5];
              data_byte[1]=cmd_string_buf[6];
              data_byte[2]=cmd_string_buf[7];
              data_byte[3]=cmd_string_buf[8];
              break;
          }
        }
        //更新
        sum -= cmd_string_buf[0];
        cmd_string_buf.pop_front();
        cmd_string_buf.push_back(current_str);
        sum += current_str;
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
                    if(new_packed_ok_len==100)
                    {
                      for(j=0;j<20;j++)
                      {
                          memcpy(&receive_byte[j],&cmd_string_buf[5*j],4);
                      }
                      mbUpdated_imu=true;
                    }


                    if(mbUpdated_imu)
                    {
                      for(j=0;j<19;j++)
                      {
                          if(cmd_string_buf[5*j+4]!=32)
                          {
                            mbUpdated_imu=false;
                            break;
                          }
                      }
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
  static bool update_theta=false;
  static bool first_update_car=true;
  //先处理imu
  {
    static float yaw_last=0.0;
    static int update_nums=0;
    boost::mutex::scoped_lock lock(mMutex_imu);
    float angle;
    if(car_status.status_imu==1 && yaw_ready )
    {
      update_theta=true;
    }
    else
    {
      update_theta=false;
    }

    if(mbUpdated_imu && car_status.status_imu==1)
    {
      //4元数转角度
      float pitch,roll,yaw;
      float q0,q1,q2,q3;
      q0 = car_status.quat[0];
      q1 = car_status.quat[1];
      q2 = car_status.quat[2];
      q3 = car_status.quat[3];
      pitch = asin(2*q1*q3 - 2*q0*q2)*57.3;
      roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
      yaw = atan2(2*(q1*q2 + q0*q3), q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
      if(!yaw_ready)
      {
        //先计算飘逸平均值初始值
        if(yaw_index==0) yaw_last = yaw;
        if(std::fabs(yaw - yaw_last)<0.01)
        {
          yaw_sum -= yaw_deltas[yaw_index];
          yaw_deltas[yaw_index] = yaw - yaw_last;
          yaw_sum += yaw_deltas[yaw_index];

          yaw_index++;
          if(yaw_index>99)
          {
            yaw_index = 0;
          }
          update_nums++;
          if(update_nums>300)
          {
            yaw_ready = true;
            update_theta=true;
            yaw_omega = yaw_sum/100.0;
            update_nums=25;
          }
        }

      }
     else
     {
       //更新飘逸速率
      // std::cout<<"oups4: "<<std::endl;
       if(car_status.encoder_delta_r == 0 && car_status.encoder_delta_l == 0)
       {
         //
         if(update_nums<25)
         {
           update_nums++;
         }
         else
         {
           if(std::fabs(yaw - yaw_last)<0.01)
           {
             yaw_sum -= yaw_deltas[yaw_index];
             yaw_deltas[yaw_index] = yaw - yaw_last;
             yaw_sum += yaw_deltas[yaw_index];

             yaw_index++;
             if(yaw_index>99)
             {
               yaw_index = 0;
             }
             yaw_omega = yaw_sum/100.0;
           }
         }
         if(debug_flag)
         {
           if((yaw - yaw_last)<-179.999)
           {
             car_status.theta += 360 + (yaw - yaw_last) - yaw_omega;
           }
           else if((yaw - yaw_last)>179.999)
           {
             car_status.theta += -360 + (yaw - yaw_last) - yaw_omega;
           }
           else
           {
             car_status.theta += yaw - yaw_last - yaw_omega;
           }
         }
       }
       else
       {
         if(update_nums > 0) update_nums--;
         //将yaw转换成360度
         if((yaw - yaw_last)<-179.9999)
         {
           car_status.theta += 360 + (yaw - yaw_last) - yaw_omega;
         }
         else if((yaw - yaw_last)>179.999)
         {
           car_status.theta += -360 + (yaw - yaw_last) - yaw_omega;
         }
         else
         {
           car_status.theta += yaw - yaw_last - yaw_omega;
         }
       }
     }

      if( car_status.theta > 360) car_status.theta -= 360;
      if( car_status.theta < 0 ) car_status.theta += 360;
      yaw_last=yaw;

      //发布IMU topic
      ros::Time current_time = ros::Time::now();
      tf::Quaternion q;
      q.setRPY(roll/180.0*PI, -pitch/180.0*PI, CarPos2D.theta/180.0*PI);
      CarIMU.header.stamp = current_time;
      CarIMU.header.frame_id = "imu";
      CarIMU.orientation.x=q.x();
      CarIMU.orientation.y=q.y();
      CarIMU.orientation.z=q.z();
      CarIMU.orientation.w=q.w();

      CarIMU.angular_velocity.x=car_status.IMU[3]* PI /180.0f;
      CarIMU.angular_velocity.y=car_status.IMU[4]* PI /180.0f;
      CarIMU.angular_velocity.z=car_status.IMU[5]* PI /180.0f;

      CarIMU.linear_acceleration.x=car_status.IMU[0]*9.8;
      CarIMU.linear_acceleration.y=car_status.IMU[1]*9.8;
      CarIMU.linear_acceleration.z=car_status.IMU[2]*9.8;

      mIMUPub.publish(CarIMU);
      mbUpdated_imu = false;

      static unsigned int ii=0;
      ii++;
     //  if(ii%50==0)
     //  {
     //    std::cout<<"IMU_acc: x "<<  car_status.IMU[0] <<" y " << car_status.IMU[1] << " z "<< car_status.IMU[2] <<std::endl;
     //    std::cout<<"IMU_gyo: x "<<  car_status.IMU[3] <<" y " << car_status.IMU[4] << " z "<< car_status.IMU[5] <<std::endl;
     //    std::cout<<"IMU_cps: x "<<  car_status.IMU[6] <<" y " << car_status.IMU[7] << " z "<< car_status.IMU[8] <<std::endl;
      //


       //  std::cout<<"status_imu:"<< car_status.status_imu<<std::endl;
       //  std::cout<<"power_imu:"<< car_status.power_imu <<std::endl;
       //  std::cout<<"roll: "<< roll<<" pitch: "<<pitch<<" yaw: "<<yaw  << " theta: " << car_status.theta <<std::endl;
       //  std::cout<<"time_stamp_imu:"<< car_status.time_stamp_imu <<std::endl;
     //  }

     //发布超声波topic
     if(car_status.distance[0]>0.1)
     {
       CarSonar1.header.stamp = current_time;
       CarSonar1.range = car_status.distance[0];
       mSonar1Pub.publish(CarSonar1);
     }
     if(car_status.distance[1]>0.1)
     {
       CarSonar2.header.stamp = current_time;
       CarSonar2.range = car_status.distance[1];
       mSonar2Pub.publish(CarSonar2);
     }
     if(car_status.distance[2]>0.1)
     {
       CarSonar3.header.stamp = current_time;
       CarSonar3.range = car_status.distance[2];
       mSonar3Pub.publish(CarSonar3);
     }
     if(car_status.distance[3]>0.1)
     {
       CarSonar4.header.stamp = current_time;
       CarSonar4.range = car_status.distance[3];
       mSonar4Pub.publish(CarSonar4);
     }
    }
  }
  //再处理car
  {
   if(mbUpdated_car && car_status.status_imu==1)
   {
     boost::mutex::scoped_lock lock(mMutex_car);
     static double theta_last=0.0;
     static unsigned int ii=0;
     ii++;
     // Time
     ros::Time current_time = ros::Time::now();

     //pose
     double delta_car,delta_x,delta_y,delta_theta,var_len,var_angle;

       if(first_update_car)
       {
         //第一次更新
         car_status.encoder_r_last = car_status.encoder_r_current;
         car_status.encoder_l_last = car_status.encoder_l_current;
         car_status.encoder_delta_r = 0;
         car_status.encoder_delta_l = 0;
         first_update_car=false;
       }
       else
       {
         car_status.encoder_delta_r = car_status.encoder_r_current - car_status.encoder_r_last;
         car_status.encoder_delta_l = car_status.encoder_l_current - car_status.encoder_l_last;
         if(car_status.encoder_delta_r > 8000) car_status.encoder_delta_r -= 2147483648;
         if(car_status.encoder_delta_r < -8000) car_status.encoder_delta_r += 2147483647;
         if(car_status.encoder_delta_l > 8000) car_status.encoder_delta_l -= 2147483648;
         if(car_status.encoder_delta_l < -8000) car_status.encoder_delta_l += 2147483647;
         car_status.encoder_delta_r = -car_status.encoder_delta_r;//方向相反
         car_status.encoder_r_last = car_status.encoder_r_current;
         car_status.encoder_l_last = car_status.encoder_l_current;
       }
       var_len=(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius);
       var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);

       delta_car=(car_status.encoder_delta_r+car_status.encoder_delta_l)/2.0f*1.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius;
       if(delta_car>0.08||delta_car<-0.08)
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
       delta_x=delta_car*cos(CarPos2D.theta* PI / 180.0f);
       delta_y=delta_car*sin(CarPos2D.theta* PI / 180.0f);

       delta_theta=car_status.theta-theta_last;
       theta_last=car_status.theta;

       if(delta_theta > 270 ) delta_theta -= 360;
       if(delta_theta < -270 ) delta_theta += 360;

       if((!update_theta)||delta_theta>30||delta_theta<-30)
       {
         delta_theta = 0;
       }
       CarPos2D.x+=delta_x;
       CarPos2D.y+=delta_y;
       CarPos2D.theta+=delta_theta;

       if(CarPos2D.theta>360.0) CarPos2D.theta-=360;
       if(CarPos2D.theta<0.0) CarPos2D.theta+=360;

       mPose2DPub.publish(CarPos2D);

       //flag
       std_msgs::Int32 flag;
       if(car_status.status_imu==1 && car_status.driver_error==0 && yaw_ready)
       {
         car_status.status=1;
       }
       else
       {
         car_status.status=0;
       }
       flag.data=car_status.status;
       //底层障碍物信息
       if((car_status.distance[0]+car_status.distance[1]+car_status.distance[2]+car_status.distance[3])>-0.8&&(car_status.distance[0]+car_status.distance[0]+car_status.distance[0]+car_status.distance[0])<1.2)
       {
         //有障碍物
         flag.data=2;
       }
       mStatusFlagPub.publish(flag);

       //Twist
       double angle_speed;
       CarTwist.linear.x = CarTwist.linear.x*0.5f + 0.5f*delta_car*50.0f;
       angle_speed=-car_status.IMU[5];
       CarTwist.angular.z = CarTwist.angular.z*0.5f + 0.5f*angle_speed * PI /180.0f;
       mTwistPub.publish(CarTwist);

       CarPower.data = car_status.power_imu*36.4f/35.55f;
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

       ros::spinOnce();

       mbUpdated_car = false;
   }
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
    speed[0]=car_status.omga_r/car_status.encoder_ppr*2.0*PI*wheel_radius;
    speed[1]=car_status.omga_l/car_status.encoder_ppr*2.0*PI*wheel_radius;
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

bool StatusPublisher::filter_speed(const int  v_in, const int r_in,int & v_out, int & r_out)
{
  //安全速度的设定依据是：100ms后再制动可以在安全距离0.2m范围外停车
  const float delay_time = 0.1; //100ms
  const float safe_distance = 0.3; //0.3m
  const float de_acc = 1.0;// m/s^2
  v_out=v_in;
  r_out=r_in;
  if(r_in==-1||r_in==1||v_in==0)
  {
    return false;
  }

  if(v_in>=1)
  {
    //前进
    float s2=2*de_acc*(car_status.distance[1]-delay_time*v_in/1000.0-safe_distance);
    if(s2<0.000001)
    {
      v_out=0;
    }
    else
    {
      v_out=std::min(v_in,(int)(sqrt(s2)*1000.0f));
      if(r_in>1 && car_status.distance[0]<=safe_distance)
      {
        //左转
        r_out=0;
      }
      else if(r_in<-1 && car_status.distance[2]<=safe_distance)
      {
        //右转
        r_out=0;
      }
    }

  }
  else if(v_in<=-1)
  {
    //后退
    float s4=2*de_acc*(car_status.distance[3]+delay_time*v_in/1000.0-safe_distance);
    if(s4<0.000001)
    {
      v_out=0;
    }
    else
    {
      v_out=std::max(v_in,(int)(-sqrt(s4)*1000.0f));
    }
  }
  return true;
}
bool StatusPublisher::isneed_faststop(void)
{
  const float safe_distance = 0.25; //0.25m

  float v=CarTwist.linear.x;
  float w=(car_status.encoder_delta_r-car_status.encoder_delta_l)*1.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius/wheel_separation;
  if(v<-0.01 && car_status.distance[3]<=safe_distance)
  {
    // 后退
    return true;
  }
  if(v>0.01)
  {
      if(car_status.distance[1]<=safe_distance) return true; //前进
      if(car_status.distance[0]<=safe_distance && w>=0.01) return true; //左转
      if(car_status.distance[2]<=safe_distance && w<=-0.01) return true; //右转
  }
  return false;
}
} //namespace xqserial_server
