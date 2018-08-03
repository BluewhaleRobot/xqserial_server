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
    car_status.encoder_ppr=1024;
    for(i=0;i<4;i++)
    {
      car_status.distance[i]=4.2;
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

   ranges_[0] = 4.2;
   ranges_[1] = 4.2;
   ranges_[2] = 4.2;
   ranges_[3] = 4.2;

   view_angles_[0] = 0.35/2.0;
   view_angles_[1] = 0.35/2.0;
   view_angles_[2] = 0.35/2.0;
   view_angles_[3] = 0.35/2.0;

   sonarTf_ready_ = false;

}

StatusPublisher::StatusPublisher(double separation,double radius,bool debugFlag)
{
    new (this)StatusPublisher();
    wheel_separation=separation;
    wheel_radius=radius;
    debug_flag=debugFlag;
    CarSonar1.header.frame_id = "sonar1";
    CarSonar1.radiation_type = 0;
    CarSonar1.field_of_view = 0.35;
    CarSonar1.min_range = 0.19;
    CarSonar1.max_range = 4.2;

    CarSonar2.header.frame_id = "sonar2";
    CarSonar2.radiation_type = 0;
    CarSonar2.field_of_view = 0.35;
    CarSonar2.min_range = 0.19;
    CarSonar2.max_range = 4.2;

    CarSonar3.header.frame_id = "sonar3";
    CarSonar3.radiation_type = 0;
    CarSonar3.field_of_view = 0.35;
    CarSonar3.min_range = 0.19;
    CarSonar3.max_range = 4.2;

    CarSonar4.header.frame_id = "sonar4";
    CarSonar4.radiation_type = 0;
    CarSonar4.field_of_view = 0.35;
    CarSonar4.min_range = 0.19;
    CarSonar4.max_range = 4.2;

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
  receive_byte=(int *)&car_status.status_car;

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
                  // std::cout<<"runup4 "<<std::endl;
                  //当前包已经处理完成，开始处理
                  boost::mutex::scoped_lock lock(mMutex_car);

                  if(new_packed_ok_len==125)
                  {
                      for(j=0;j<25;j++)
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
                          mbUpdated_car=false;
                          car_status.encoder_ppr=1024;
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
                    boost::mutex::scoped_lock lock1(mMutex_imu);
                    boost::mutex::scoped_lock lock2(mMutex_range);
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

       if(ii%5==0)
       {
         boost::mutex::scoped_lock lock(mMutex_range);
         //发布超声波topic
         if(car_status.distance[0]>0.1&&car_status.distance[0]<4.30)
         {
           CarSonar1.header.stamp = current_time;
           CarSonar1.range = car_status.distance[0];
           mSonar1Pub.publish(CarSonar1);
           ranges_[0] = CarSonar1.range;
         }
         if(car_status.distance[1]>0.1&&car_status.distance[0]<4.3)
         {
           CarSonar2.header.stamp = current_time;
           CarSonar2.range = car_status.distance[1];
           mSonar2Pub.publish(CarSonar2);
           ranges_[1] = CarSonar2.range;
         }
         if(car_status.distance[2]>0.1&&car_status.distance[0]<4.3)
         {
           CarSonar3.header.stamp = current_time;
           CarSonar3.range = car_status.distance[2];
           mSonar3Pub.publish(CarSonar3);
           ranges_[2] = CarSonar3.range;
         }
         if(car_status.distance[3]>0.1&&car_status.distance[0]<4.3)
         {
           CarSonar4.header.stamp = current_time;
           CarSonar4.range = car_status.distance[3];
           mSonar4Pub.publish(CarSonar4);
           ranges_[3] = CarSonar4.range;
         }
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
       if(car_status.status_imu==1  && yaw_ready)
       {
         car_status.status=1;
       }
       else
       {
         car_status.status=0;
       }
       flag.data=car_status.status;
       //底层障碍物信息
       //超声波障碍物信息
       int barArea_nums=0;
       int clearArea_nums=0;

       float ranges[4],view_angles[4],tf_angles[4],tf_xs[4],tf_ys[4];
       if(this->getSonarTf(tf_angles,tf_xs,tf_ys))
       {
         float x0,y0,r0;
         this->getSonarData(ranges,view_angles);
         //模块1
         // if(ranges[0]>0 && ranges[0]<0.8)
         // {
         //   barArea_nums += 2*((int)(ranges[0]*tan(view_angles[0])/kinect_stepsize_)) + 1;
         // }
         x0 = ranges[0]*cos(tf_angles[0]) + tf_xs[0];
         y0 = ranges[0]*sin(tf_angles[0]) + tf_ys[0];
         r0 = x0*x0+y0*y0;
         if(r0<0.26*0.26)
         {
           barArea_nums += 2*((int)(ranges[0]*tan(view_angles[0])/kinect_stepsize_)) + 1;
           flag.data=2;
         }
         else{
           clearArea_nums+=25;
         }
         //模块2
         // if(ranges[1]>0 && ranges[1]<0.8)
         // {
         //   barArea_nums += 2*((int)(ranges[1]*tan(view_angles[1])/kinect_stepsize_)) + 1;
         // }
         x0 = ranges[1]*cos(tf_angles[1]) + tf_xs[1];
         y0 = ranges[1]*sin(tf_angles[1]) + tf_ys[1];
         r0 = x0*x0+y0*y0;
         if(r0<0.26*0.26)
         {
           barArea_nums += 2*((int)(ranges[1]*tan(view_angles[1])/kinect_stepsize_)) + 1;
           flag.data=2;
         }
         else{
           clearArea_nums+=15;
         }
         //模块3
         // if(ranges[2]>0 && ranges[2]<0.8)
         // {
         //   barArea_nums += 2*((int)(ranges[2]*tan(view_angles[2])/kinect_stepsize_)) + 1;
         // }
         x0 = ranges[2]*cos(tf_angles[2]) + tf_xs[2];
         y0 = ranges[2]*sin(tf_angles[2]) + tf_ys[2];
         r0 = x0*x0+y0*y0;
         if(r0<0.26*0.26)
         {
           barArea_nums += 2*((int)(ranges[2]*tan(view_angles[2])/kinect_stepsize_)) + 1;
           flag.data=2;
         }
         else{
           clearArea_nums+=25;
         }
         //模块4
         x0 = ranges[3]*cos(tf_angles[3]) + tf_xs[3];
         y0 = ranges[3]*sin(tf_angles[3]) + tf_ys[3];
         r0 = x0*x0+y0*y0;
         if(r0<0.44*0.44)
         {
           flag.data=2;
         }

       }
       mStatusFlagPub.publish(flag);
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
         float x0,y0,r0;
         //模块1
         x0 = ranges[0]*cos(tf_angles[0]) + tf_xs[0];
         y0 = ranges[0]*sin(tf_angles[0]) + tf_ys[0];
         r0 = x0*x0+y0*y0;
         if(r0<0.26*0.26)
         {
           int nums1 = (int)(ranges[0]*tan(view_angles[0])/kinect_stepsize_);
           for(int i = -nums1;i<=nums1;i++,++bariter_x,++bariter_y,++bariter_z)
           {
             float x0 ,y0;
             x0 = ranges[0];
             y0 = i*kinect_stepsize_;
             *bariter_x = x0*cos(tf_angles[0]) - y0*sin(tf_angles[0]) + tf_xs[0] - kinect_x_;
             *bariter_y = x0*sin(tf_angles[0]) + y0*cos(tf_angles[0]) + tf_ys[0];
             *bariter_z = 0.15;
           }
         }
         //模2
         x0 = ranges[1]*cos(tf_angles[1]) + tf_xs[1];
         y0 = ranges[1]*sin(tf_angles[1]) + tf_ys[1];
         r0 = x0*x0+y0*y0;
         if(r0<0.26*0.26)
         {
           int nums1 = (int)(ranges[1]*tan(view_angles[1])/kinect_stepsize_);
           for(int i = -nums1;i<=nums1;i++,++bariter_x,++bariter_y,++bariter_z)
           {
             float x0 ,y0;
             x0 = ranges[1];
             y0 = i*kinect_stepsize_;
             *bariter_x = x0*cos(tf_angles[1]) - y0*sin(tf_angles[1]) + tf_xs[1] - kinect_x_;
             *bariter_y = x0*sin(tf_angles[1]) + y0*cos(tf_angles[1]) + tf_ys[1];
             *bariter_z = 0.15;
           }
         }
         //模块3
         x0 = ranges[2]*cos(tf_angles[2]) + tf_xs[2];
         y0 = ranges[2]*sin(tf_angles[2]) + tf_ys[2];
         r0 = x0*x0+y0*y0;
         if(r0<0.26*0.26)
         {
           int nums1 = (int)(ranges[2]*tan(view_angles[2])/kinect_stepsize_);
           for(int i = -nums1;i<=nums1;i++,++bariter_x,++bariter_y,++bariter_z)
           {
             float x0 ,y0;
             x0 = ranges[2];
             y0 = i*kinect_stepsize_;
             *bariter_x = x0*cos(tf_angles[2]) - y0*sin(tf_angles[2]) + tf_xs[2] - kinect_x_;
             *bariter_y = x0*sin(tf_angles[2]) + y0*cos(tf_angles[2]) + tf_ys[2];
             *bariter_z = 0.15;
           }
         }
         if(ii%5==0)
         {
           pub_barpoint_cloud_.publish(barcloud_msg);
         }
       }
       if(clearArea_nums>0)
       {
         //发布雷区
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
         float x0,y0,r0;
         //模块1
         x0 = ranges[0]*cos(tf_angles[0]) + tf_xs[0];
         y0 = ranges[0]*sin(tf_angles[0]) + tf_ys[0];
         r0 = x0*x0+y0*y0;
         if(r0>=0.26*0.26)
         {
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.25 - kinect_x_;
             *cleariter_y=0.1+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.2 - kinect_x_;
             *cleariter_y=0.1+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.15 - kinect_x_;
             *cleariter_y=0.1+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.1 - kinect_x_;
             *cleariter_y=0.1+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.05 - kinect_x_;
             *cleariter_y=0.1+k*0.05;
             *cleariter_z=0.0;
           }
         }
         //模块2
         x0 = ranges[1]*cos(tf_angles[1]) + tf_xs[1];
         y0 = ranges[1]*sin(tf_angles[1]) + tf_ys[1];
         r0 = x0*x0+y0*y0;
         if(r0>=0.26*0.26)
         {
           for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.25 - kinect_x_;
             *cleariter_y=-0.05+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.2 - kinect_x_;
             *cleariter_y=-0.05+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.15 - kinect_x_;
             *cleariter_y=-0.05+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.1 - kinect_x_;
             *cleariter_y=-0.05+k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.05 - kinect_x_;
             *cleariter_y=-0.05+k*0.05;
             *cleariter_z=0.0;
           }
         }
         //模块3
         x0 = ranges[2]*cos(tf_angles[2]) + tf_xs[2];
         y0 = ranges[2]*sin(tf_angles[2]) + tf_ys[2];
         r0 = x0*x0+y0*y0;
         if(r0>=0.26*0.26)
         {
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.25 - kinect_x_;
             *cleariter_y=-0.10-k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.2 - kinect_x_;
             *cleariter_y=-0.10-k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.15 - kinect_x_;
             *cleariter_y=-0.10-k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.10 - kinect_x_;
             *cleariter_y=-0.10-k*0.05;
             *cleariter_z=0.0;
           }
           for(int k=0;k<5;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
           {
             *cleariter_x=0.05 - kinect_x_;
             *cleariter_y=-0.10-k*0.05;
             *cleariter_z=0.0;
           }
         }
         if(ii%5==0)
         {
           pub_clearpoint_cloud_.publish(clearcloud_msg);
         }
       }

       //Twist
       double angle_speed;
       CarTwist.linear.x = CarTwist.linear.x*0.5f + 0.5f*delta_car*50.0f;
       angle_speed=car_status.IMU[5];
       CarTwist.angular.z = CarTwist.angular.z*0.5f + 0.5f*angle_speed * PI /180.0f;
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

void StatusPublisher::getSonarData(float (&ranges)[4],float (&view_angles)[4])
{
  boost::mutex::scoped_lock lock(mMutex_range);
  for(int i =0 ;i<4;i++)
  {
    ranges[i] = ranges_[i];
    view_angles[i] = view_angles_[i];
  }
}

void StatusPublisher::setSonarTf(tf::StampedTransform &transform1,tf::StampedTransform &transform2,tf::StampedTransform &transform3,tf::StampedTransform &transform4)
{
  boost::mutex::scoped_lock lock(mMutex_range);
  sonarTf_ready_ = true;
  transform1_ = transform1;
  transform2_ = transform2;
  transform3_ = transform3;
  transform4_ = transform4;
}

bool StatusPublisher::getSonarTf(float (&tf_angles)[4],float (&tf_xs)[4],float (&tf_ys)[4])
{
  boost::mutex::scoped_lock lock(mMutex_range);
  if(!sonarTf_ready_)
  {
   return false;
  }
  tf_xs [0] = transform1_.getOrigin().x();
  tf_ys [0] = transform1_.getOrigin().y();
  tf_angles[0] = tf::getYaw(transform1_.getRotation()); //0 2pi

  tf_xs [1] = transform2_.getOrigin().x();
  tf_ys [1] = transform2_.getOrigin().y();
  tf_angles[1] = tf::getYaw(transform2_.getRotation()); //0 2pi

  tf_xs [2] = transform3_.getOrigin().x();
  tf_ys [2] = transform3_.getOrigin().y();
  tf_angles[2] = tf::getYaw(transform3_.getRotation()); //0 2pi

  tf_xs [3] = transform4_.getOrigin().x();
  tf_ys [3] = transform4_.getOrigin().y();
  tf_angles[3] = tf::getYaw(transform4_.getRotation()); //0 2pi
  return true;
}

void StatusPublisher::setBarparams(double kinect_x,double kinect_stepsize)
{
  kinect_x_ = kinect_x;
  kinect_stepsize_ = kinect_stepsize;
}

} //namespace xqserial_server
