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
    wheel_separation=0.37;
    wheel_radius=0.06;

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
    for(i=0;i<24;i++)
    {
        status[i]=0;
    }
    car_status.encoder_ppr=1000;

   mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D",1,true);
   mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag",1,true);
   mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist",1,true);
   mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
   mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);

   // 蓝海的数据
   mTargetIndexPub = mNH.advertise<std_msgs::Int32>("xqserial_server/targetIndex",1,true);
   mTargetAnglePub = mNH.advertise<std_msgs::Int32>("xqserial_server/targetAngle",1,true);
   mIMUPub = mNH.advertise<sensor_msgs::Imu>("xqserial_server/IMU", 1, true);
   car_status.poseID=-1;
   car_status.power_imu=12.0;
   debug_flag=false;
  /* static tf::TransformBroadcaster br;
   tf::Quaternion q;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.13) );//摄像头距离地面高度13cm
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
   */

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
}

void StatusPublisher::Update_car(const char data[], unsigned int len)
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
    //int ii=0;
    //boost::mutex::scoped_lock lock(mMutex);

    // if(len<119)
    // {
    //   std::cout<<"len0:"<<len<<std::endl;
    //   current_str=data[0];
    //   std::cout<<(unsigned int)current_str<<std::endl;
    // }
    for(i=0;i<len;i++)
    {
        current_str=data[i];
       // unsigned int temp=(unsigned int)current_str;
       //std::cout<<len<<std::endl;
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
                    //std::cout<<"runup4 "<<new_packed_ok_len <<std::endl;
                    //当前包已经处理完成，开始处理
                    boost::mutex::scoped_lock lock(mMutex_car);
                    if(new_packed_ok_len==40)
                    {

                        for(j=0;j<8;j++)
                        {
                            memcpy(&receive_byte[j],&cmd_string_buf[5*j],4);
                        }
                        mbUpdated_car=true;
                    }

                    if(mbUpdated_car)
                    {
                      for(j=0;j<7;j++)
                      {
                          if(cmd_string_buf[5*j+4]!=32)
                          {
                            //   std::cout<<"len:"<< len <<std::endl;
                            //   std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
                            //   for(j=0;j<115;j++)
                            //   {
                            //     current_str=cmd_string_buf[j];
                            //     std::cout<<(unsigned int)current_str<<std::endl;
                            //   }
                            mbUpdated_car=false;
                            car_status.encoder_ppr=1000;
                            break;
                          }
                      }
                    }
                    // if(mbUpdated_car&&(car_status.encoder_delta_car>3000||car_status.encoder_delta_car<-3000))
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
                    new_packed_ok_len=0;
                    new_packed_len=0;
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
void StatusPublisher::Update_imu(const char data[], unsigned int len)
{
  // if(len <1) return;
  // static char data2[1024];
  // static int len2=0;
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
    //int ii=0;
    //boost::mutex::scoped_lock lock(mMutex);

    // if(len<119)
    // {
    //   std::cout<<"len0:"<<len<<std::endl;
    //   current_str=data[0];
    //   std::cout<<(unsigned int)current_str<<std::endl;
    // }
    for(i=0;i<len;i++)
    {
        current_str=data[i];
       // unsigned int temp=(unsigned int)current_str;
       // std::cout<<temp<<std::endl;
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
                    if(new_packed_ok_len==80)
                    {
                      for(j=0;j<16;j++)
                      {
                          memcpy(&receive_byte[j+8],&cmd_string_buf[5*j],4);
                      }
                      mbUpdated_imu=true;
                    }


                    if(mbUpdated_imu)
                    {
                      for(j=0;j<15;j++)
                      {
                          if(cmd_string_buf[5*j+4]!=32)
                          {
                            //   std::cout<<"len:"<< len <<std::endl;
                            //   std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
                            //   for(j=0;j<115;j++)
                            //   {
                            //     current_str=cmd_string_buf[j];
                            //     std::cout<<(unsigned int)current_str<<std::endl;
                            //   }
                            mbUpdated_imu=false;
                            break;
                          }
                      }
                    }
                    // if(mbUpdated_imu&&(car_status.encoder_delta_car>3000||car_status.encoder_delta_car<-3000))
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
                    new_packed_ok_len=0;
                    new_packed_len=0;
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
     static bool update_theta=false;
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
             if(update_nums>600)
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
         mIMUPub.publish(CarIMU);
         mbUpdated_imu = false;

         static unsigned int ii=0;
         ii++;
        //  if(ii%50==0)
        //  {
        //    //std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
         //
        //    std::cout<<"status_imu:"<< car_status.status_imu<<std::endl;
        //    std::cout<<"power_imu:"<< car_status.power_imu <<std::endl;
        //    std::cout<<"roll: "<< roll<<" pitch: "<<pitch<<" yaw: "<<yaw  << " theta: " << car_status.theta <<std::endl;
        //    std::cout<<"time_stamp_imu:"<< car_status.time_stamp_imu <<std::endl;
        //  }
       }
     }
     //再处理car
     {
       if(mbUpdated_car && car_status.status_car==1)
       {
         boost::mutex::scoped_lock lock(mMutex_car);
         static double theta_last=0.0;
         static unsigned int ii=0;
         ii++;
         ros::Time current_time = ros::Time::now();

         //pose
         double delta_car,delta_x,delta_y,delta_theta,var_len,var_angle;


         var_len=(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius);
         var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);

         delta_car=(car_status.encoder_delta_r+car_status.encoder_delta_l)/2.0f*1.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius;
         if(delta_car>0.05||delta_car<-0.05)
         {
           // std::cout<<"get you!"<<std::endl;
           delta_car = 0;
         }
         car_status.encoder_delta_r =  (car_status.encoder_delta_r+car_status.encoder_delta_l)/2.0;
         car_status.omga_r = car_status.encoder_delta_r*50;
         car_status.omga_l = car_status.encoder_delta_l*50;

        //  if(ii%50==0||car_status.encoder_delta_car>3000||car_status.encoder_delta_car<-3000)
        //  {
        //    //std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
        //    std::cout<<"ppr:"<< car_status.encoder_ppr <<std::endl;
        //    std::cout<<"delta_encoder_r:"<< car_status.encoder_delta_r <<std::endl;
        //    std::cout<<"delta_encoder_l:"<< car_status.encoder_delta_l <<std::endl;
        //    std::cout<<"time_stamp_car:"<< car_status.time_stamp_car <<std::endl;
        //    std::cout<<"poseID:"<< car_status.poseID <<std::endl;
        //    std::cout<<"poseAngle:"<< car_status.poseAngle <<std::endl;
        //  }
         delta_x=delta_car*cos(CarPos2D.theta* PI / 180.0f);
         delta_y=delta_car*sin(CarPos2D.theta* PI / 180.0f);

         delta_theta=car_status.theta-theta_last;
         theta_last=car_status.theta;

         if( delta_theta > 270)
         {
            delta_theta -= 360;
         }

         if( delta_theta < -270 )
         {
            delta_theta += 360;
         }

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
         if(car_status.status_imu==1 && car_status.status_car==1 && yaw_ready)
         {
           car_status.status=1;
         }
         else
         {
           car_status.status=0;
         }

         flag.data=car_status.status;
         //底层障碍物信息
         mStatusFlagPub.publish(flag);

         //Twist
         double angle_speed;
         CarTwist.linear.x=delta_car*50.0f;
         angle_speed=-car_status.IMU[5];
         CarTwist.angular.z=angle_speed * PI /180.0f;
         mTwistPub.publish(CarTwist);

         CarPower.data = car_status.power_imu;
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

         // 蓝海的需求
         std_msgs::Int32 target_index;
         //std::cout << "here" << std::endl;
         //std::cout << car_status.poseID << std::endl;
         target_index.data = car_status.poseID;
         mTargetIndexPub.publish(target_index);
         std_msgs::Int32 target_angle;
         target_angle.data = car_status.poseAngle;
         mTargetAnglePub.publish(target_angle);

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

} //namespace xqserial_server
