#include "DiffDriverController.h"
#include <time.h>

namespace xqserial_server
{


DiffDriverController::DiffDriverController()
{
    max_wheelspeed=2.0;
    cmd_topic="cmd_vel";
    xq_status=new StatusPublisher();
    cmd_serial=NULL;
    MoveFlag=true;
}

DiffDriverController::DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_)
{
    MoveFlag=true;
    max_wheelspeed=max_speed_;
    cmd_topic=cmd_topic_;
    xq_status=xq_status_;
    cmd_serial=cmd_serial_;
    barDetectFlag = true;
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
    char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x43};
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,5);
    }
  }
}

void DiffDriverController::updateBarDetectFlag(const std_msgs::Bool& DetectFlag)
{
  barDetectFlag = DetectFlag.data;
  if(DetectFlag.data)
  {
    //下发底层红外开启命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x01};
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);
    }
  }
  else
  {
    //下发底层红外禁用命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x00};
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);
    }
  }
}

void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
{
    //std::cout<<"oups1 "<<std::endl;
    static time_t t1=time(NULL),t2;
    int i=0,wheel_ppr=1;
    double separation=0,radius=0,speed_lin=0,speed_ang=0,speed_temp[2];
    char speed[2]={0,0};//左一右二
    char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};

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
    speed_temp[1]=scale*(speed_lin+speed_ang/2)/max_wheelspeed*100.0;
    speed_temp[1]=std::min(speed_temp[1],100.0);
    speed_temp[1]=std::max(-100.0,speed_temp[1]);

    speed_temp[0]=scale*(speed_lin-speed_ang/2)/max_wheelspeed*100.0;
    speed_temp[0]=std::min(speed_temp[0],100.0);
    speed_temp[0]=std::max(-100.0,speed_temp[0]);

  //std::cout<<"radius "<<radius<<std::endl;
  //std::cout<<"ppr "<<wheel_ppr<<std::endl;
  //std::cout<<"max_speed "<<max_wheelspeed<<std::endl;
  //std::cout<<"speed_lin "<<speed_lin<<std::endl;
  //  command.linear.x/
    for(i=0;i<2;i++)
    {
     speed[i]=(int8_t)speed_temp[i];
     //std::cout<<"i "<< i <<" speed_temp[i]  "<<speed_temp[i] << " speed[i] " << speed[i]<<std::endl;

     if(speed[i]<0)
     {
         if(speed[i]>-10) speed[i]=-10;
         cmd_str[5+i*2]=(char)0x42;//B
         cmd_str[9+i*2]=-speed[i];
         cmd_str[5+i*2+1]=(char)0x42;//B
         cmd_str[9+i*2+1]=-speed[i];
     }
     else if(speed[i]>0)
     {
         if(speed[i]<10) speed[i]=10;
         cmd_str[5+i*2]=(char)0x46;//F
         cmd_str[9+i*2]=speed[i];
         cmd_str[5+i*2+1]=(char)0x46;//F
         cmd_str[9+i*2+1]=speed[i];
     }
     else
     {
         cmd_str[5+i*2]=(char)0x53;//S
         cmd_str[9+i*2]=(char)0x00;
         cmd_str[5+i*2+1]=(char)0x53;//S
         cmd_str[9+i*2+1]=(char)0x00;
     }
    }


    if(xq_status->get_status()==2 && barDetectFlag)
    {
      float ranges[4],view_angles[4];
      xq_status->getSonarData(ranges,view_angles);
      //有障碍物
      if(ranges[0]>0.1&&ranges[0]<0.4&&cmd_str[5]==(char)0x46)
      {
        cmd_str[5]=(char)0x53;
        cmd_str[6]=(char)0x53;
      }

      if(ranges[3]>0.1&&ranges[3]<0.4&&cmd_str[5]==(char)0x42)
      {
        cmd_str[5]=(char)0x53;
        cmd_str[6]=(char)0x53;
      }

      if(ranges[1]>0.1&&ranges[1]<0.4&&cmd_str[5]==(char)0x46)
      {
        cmd_str[7]=(char)0x53;
        cmd_str[8]=(char)0x53;
      }

      if(ranges[2]>0.1&&ranges[2]<0.4&&cmd_str[5]==(char)0x42)
      {
        cmd_str[7]=(char)0x53;
        cmd_str[8]=(char)0x53;
      }
    }

    boost::mutex::scoped_lock lock(mMutex);
    if(!MoveFlag)
    {
      cmd_str[5]=(char)0x53;
      cmd_str[6]=(char)0x53;
      cmd_str[7]=(char)0x53;
      cmd_str[8]=(char)0x53;
    }
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,13);
    }

   // command.linear.x
}






















}
