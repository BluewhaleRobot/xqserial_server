#include "DiffDriverController.h"
#include <time.h>

namespace xqserial_server
{


DiffDriverController::DiffDriverController()
{
    max_wheelspeed=4.4;
    cmd_topic="/cmd_vel";
    xq_status=new StatusPublisher();
    cmd_serial_car=NULL;
    cmd_serial_imu=NULL;
    MoveFlag=true;
}

DiffDriverController::DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_car_,CallbackAsyncSerial* cmd_serial_imu_)
{
    MoveFlag=true;
    BarFlag=true;
    max_wheelspeed=max_speed_;
    cmd_topic=cmd_topic_;
    xq_status=xq_status_;
    cmd_serial_car=cmd_serial_car_;
    cmd_serial_imu=cmd_serial_imu_;

    speed_last_[0] = 0.0;
    speed_last_[1] = 0.0;

}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/globalMoveFlag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    ros::spin();
}
void DiffDriverController::updateMoveFlag(const std_msgs::Bool& moveFlag)
{
  boost::mutex::scoped_lock lock(mMutex);
  MoveFlag=moveFlag.data;

}
void DiffDriverController::imuCalibration(const std_msgs::Bool& calFlag)
{
  if(calFlag.data)
  {
    //下发底层ｉｍｕ标定命令
    boost::mutex::scoped_lock lock(mMutex);
    char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x43};
    if(NULL!=cmd_serial_imu)
    {
        cmd_serial_imu->write(cmd_str,5);
    }
  }
}
void DiffDriverController::updateBarDetectFlag(const std_msgs::Bool& DetectFlag)
{
  //避障传感器应急处理标志
  boost::mutex::scoped_lock lock(mMutex);
  BarFlag=DetectFlag.data;
  if(DetectFlag.data)
  {
    //下发底层红外开启命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x01};
    if(NULL!=cmd_serial_imu)
    {
        cmd_serial_imu->write(cmd_str,6);
    }
  }
  else
  {
    //下发底层红外禁用命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x00};
    if(NULL!=cmd_serial_imu)
    {
        cmd_serial_imu->write(cmd_str,6);
    }
  }
}

void DiffDriverController::sendcmd2()
{
  boost::mutex::scoped_lock lock(mMutex);
  if(xq_status->get_status()==0) return;//底层还在初始化
  char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
  float tf_angles[4],tf_xs[4],tf_ys[4],x0,y0,r0;
  char speed[2]={0,0};//右一左二
  //超声波障碍物信息
  if(xq_status->getSonarTf(tf_angles,tf_xs,tf_ys))
  {
    float ranges_[4],view_angles_[4];
    xq_status->getSonarData(ranges_,view_angles_);
    if(BarFlag)
    {
      //模块1
      x0 = ranges_[0]*cos(tf_angles[0]) + tf_xs[0];
      y0 = ranges_[0]*sin(tf_angles[0]) + tf_ys[0];
      r0 = x0*x0+y0*y0;
      if(r0<0.26*0.26)
      {
        //左轮不能前进
        if(speed_last_[1]>0)
        {
            speed_last_[1] = 0.0;
        }
      }

      //模块2
      x0 = ranges_[1]*cos(tf_angles[1]) + tf_xs[1];
      y0 = ranges_[1]*sin(tf_angles[1]) + tf_ys[1];
      r0 = x0*x0+y0*y0;
      if(r0<0.26*0.26)
      {
        //左轮不能前进
        if(speed_last_[1]>0)
        {
            speed_last_[1] = 0.0;
        }
        //右轮不能前进
        if(speed_last_[0]>0)
        {
            speed_last_[0] = 0.0;
        }
      }

      //模块3
      x0 = ranges_[2]*cos(tf_angles[2]) + tf_xs[2];
      y0 = ranges_[2]*sin(tf_angles[2]) + tf_ys[2];
      r0 = x0*x0+y0*y0;
      if(r0<0.26*0.26)
      {
        //右轮不能前进
        if(speed_last_[0]>0)
        {
            speed_last_[0] = 0.0;
        }
      }
    }
    //后退的模块4
    x0 = ranges_[3]*cos(tf_angles[3]) + tf_xs[3];
    y0 = ranges_[3]*sin(tf_angles[3]) + tf_ys[3];
    r0 = x0*x0+y0*y0;
    if(r0<0.44*0.44)
    {
      //不能后退
      for(int i=0;i<2;i++)
      {
       if(speed_last_[i]<0)
       {
           speed_last_[i] = 0.0;//S
       }
      }
    }
  }

  for(int i=0;i<2;i++)
  {
   speed[i]=(int8_t)speed_last_[i];
   if(speed[i]<0)
   {
       cmd_str[5+i]=(char)0x42;//B
       cmd_str[9+i]=-speed[i];
   }
   else if(speed[i]>0)
   {
       cmd_str[5+i]=(char)0x46;//F
       cmd_str[9+i]=speed[i];
   }
   else
   {
       cmd_str[5+i]=(char)0x53;//S
       cmd_str[9+i]=(char)0x00;
   }
  }
  if(!MoveFlag)
  {
    cmd_str[5]=(char)0x53;
    cmd_str[6]=(char)0x53;
    cmd_str[9]=(char)0x00;
    cmd_str[10]=(char)0x00;
  }
  if(NULL!=cmd_serial_car)
  {
      cmd_serial_car->write(cmd_str,13);
  }
}

void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
{
    static time_t t1=time(NULL),t2;
    int i=0,wheel_ppr=1;
    double separation=0,radius=0,speed_lin=0,speed_ang=0,speed_temp[2];

    if(xq_status->get_status()==0) return;//底层还在初始化
    separation=xq_status->get_wheel_separation();
    radius=xq_status->get_wheel_radius();
    wheel_ppr=xq_status->get_wheel_ppr();
    //转换速度单位，由米转换成转
    speed_lin=command.linear.x/(2.0*PI*radius);
    speed_ang=command.angular.z*separation/(2.0*PI*radius);

    float scale=std::max(std::abs(speed_lin+speed_ang/2.0),std::abs(speed_lin-speed_ang/2.0))/max_wheelspeed;
    if(scale>1.0)
    {
      scale=1.0/scale;
    }
    else
    {
      scale=1.0;
    }
    //转出最大速度百分比,并进行限幅
    speed_temp[0]=scale*(speed_lin+speed_ang/2)/max_wheelspeed*100.0;
    speed_temp[0]=std::min(speed_temp[0],100.0);
    speed_temp[0]=std::max(-100.0,speed_temp[0]);

    speed_temp[1]=scale*(speed_lin-speed_ang/2)/max_wheelspeed*100.0;
    speed_temp[1]=std::min(speed_temp[1],100.0);
    speed_temp[1]=std::max(-100.0,speed_temp[1]);

    speed_last_[0] = speed_temp[0];
    speed_last_[1] = speed_temp[1];

    this->sendcmd2();
   // command.linear.x
}


}
