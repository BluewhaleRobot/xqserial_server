#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>

#define DISABLE 0
#define ENABLE 1

namespace xqserial_server
{

StatusPublisher::StatusPublisher()
{
    mbUpdated=false;
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
    for(i=0;i<23;i++)
    {
        status[i]=0;
    }
    car_status.encoder_ppr=4*12*64;

   mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D",1,true);
   mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag",1,true);
   mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist",1,true);
   mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
   mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);

  /* static tf::TransformBroadcaster br;
   tf::Quaternion q;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.13) );//摄像头距离地面高度13cm
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
   */

}

StatusPublisher::StatusPublisher(double separation,double radius)
{
    new (this)StatusPublisher();
    wheel_separation=separation;
    wheel_radius=radius;
}

void StatusPublisher::Update(const char data[], unsigned int len)
{
    boost::mutex::scoped_lock lock(mMutex);
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
                    //当前包已经处理完成，开始处理
                    if(new_packed_ok_len==115)
                    {
                        for(j=0;j<23;j++)
                        {
                            memcpy(&receive_byte[j],&cmd_string_buf[5*j],4);
                        }
                    }
                    else if(new_packed_ok_len==95)
                    {
                        for(j=0;j<19;j++)
                        {
                            memcpy(&receive_byte[j],&cmd_string_buf[5*j],4);
                        }
                    }
                    mbUpdated=true;
                    //ii++;
                    //std::cout << ii << std::endl;
                    new_packed_ok_len=0;
                }
            }

        }

    }
    return;
}


void StatusPublisher::Refresh()
{

    //std::cout<<"runR"<< mbUpdated<<std::endl;
    if(mbUpdated)
    {
        // Time
        ros::Time current_time = ros::Time::now();
        //pose
        double delta_car,delta_x,delta_y,delta_theta,var_len,var_angle;

        var_len=(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius);
        var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);

        delta_car=car_status.encoder_delta_car*1.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius;
        delta_x=delta_car*cos(CarPos2D.theta* PI / 180.0f);
        delta_y=delta_car*sin(CarPos2D.theta* PI / 180.0f);

        CarPos2D.x+=delta_x;
        CarPos2D.y+=delta_y;
        delta_theta=car_status.theta-CarPos2D.theta;
        CarPos2D.theta=car_status.theta;

        mPose2DPub.publish(CarPos2D);

        //flag
        std_msgs::Int32 flag;
        flag.data=car_status.status;
        mStatusFlagPub.publish(flag);

        //Twist
        double angle_speed;
        CarTwist.linear.x=delta_car*50.0f;
        angle_speed=-car_status.IMU[5];
        CarTwist.angular.z=angle_speed * PI /180.0f;
        mTwistPub.publish(CarTwist);

        CarPower.data = car_status.power;
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
        boost::mutex::scoped_lock lock(mMutex);
        mbUpdated = false;
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
