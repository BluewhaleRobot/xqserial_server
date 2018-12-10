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
    last_ordertime=ros::WallTime::now();
    DetectFlag_=true;
}

DiffDriverController::DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_)
{
    MoveFlag=true;
    max_wheelspeed=max_speed_;
    cmd_topic=cmd_topic_;
    xq_status=xq_status_;
    cmd_serial=cmd_serial_;
    speed_debug[0]=0.0;
    speed_debug[1]=0.0;
    last_ordertime=ros::WallTime::now();
    DetectFlag_=true;
    stopFlag_ = false;
    mgalileoCmdsPub_ = mNH_.advertise<galileo_serial_server::GalileoNativeCmds>("/galileo/cmds", 0, true);
    back_touch_falg_ = false;
    last_touchtime_ = ros::WallTime::now();
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/globalMoveFlag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    ros::Subscriber sub5 = nodeHandler.subscribe("/move_base/StatusFlag", 1, &DiffDriverController::updateStopFlag,this);
    ros::Subscriber sub6 = nodeHandler.subscribe("/galileo/status", 1, &DiffDriverController::UpdateNavStatus, this);
    ros::spin();
}

void DiffDriverController::updateStopFlag(const std_msgs::Int32& fastStopmsg)
{
  boost::mutex::scoped_lock lock(mMutex);
  if(fastStopmsg.data == 2)
  {
    stopFlag_ = true;
  }
  else
  {
    stopFlag_ = false;
  }
}

void DiffDriverController::updateMoveFlag(const std_msgs::Bool& moveFlag)
{
  boost::mutex::scoped_lock lock(mMutex);
  MoveFlag=moveFlag.data;
  last_ordertime=ros::WallTime::now();
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
  if(DetectFlag.data)
  {
    //下发底层红外开启命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x01};
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);
    }
    DetectFlag_=true;
  }
  else
  {
    //下发底层红外禁用命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x00};
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);
    }
    DetectFlag_=false;
  }
}

geometry_msgs::Twist DiffDriverController::get_cmdTwist(void)
{
  boost::mutex::scoped_lock lock(mMutex);
  return cmdTwist_;
}
void DiffDriverController::sendcmd2(const geometry_msgs::Twist &command)
{
  {
    boost::mutex::scoped_lock lock(mMutex);
    cmdTwist_ = command;
  }

  geometry_msgs::Twist  cmdTwist = cmdTwist_;

  float cmd_v = cmdTwist.linear.x;
  if(DetectFlag_)
  {
    //超声波预处理
    double distances[2]={0.0,0.0},distance=0;
    xq_status->get_distances(distances);
    distance=std::min(distances[0],distances[1]);

    if(distance>=0.2001 && distance<=0.45)
    {
        float k=0.4;
        float max_v = std::max(0.0,k*std::sqrt(distance-0.35)); //根据当前距离设置线速度最大值

        geometry_msgs::Twist  carTwist = xq_status->get_CarTwist();
        if(max_v<0.01 && carTwist.linear.x<0.1 && carTwist.linear.x>-0.1 && cmdTwist.linear.x> 0.01)
        {
            if(distances[0]>(distances[1]+0.05) )
            {
              if(cmdTwist.angular.z<0.005)
              {
                //可以右转
                cmdTwist.angular.z = std::min(-0.1,cmdTwist.angular.z);
              }
              else
              {
                cmdTwist.angular.z = 0.1;
              }
            }
            else if(distances[1]>(distances[0]+0.05)  && cmdTwist.angular.z > -0.01)
            {
              if(cmdTwist.angular.z > -0.005)
              {
                //可以左转
                cmdTwist.angular.z = std::max(0.1,cmdTwist.angular.z);
              }
              else
              {
                cmdTwist.angular.z = -0.1;
              }
            }
        }
        if(cmd_v >= max_v)
        {
          cmd_v = max_v;
        }
    }
  }
  cmdTwist.linear.x = cmd_v;
  sendcmd(cmdTwist);
}
void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
{

    static time_t t1=time(NULL),t2;
    int i=0,wheel_ppr=1;
    double separation=0,radius=0,speed_lin=0,speed_ang=0,speed_temp[2];
    char speed[2]={0,0};//右一左二
    char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};

    if(xq_status->get_status()==0) return;//底层还在初始化
    separation=xq_status->get_wheel_separation();
    radius=xq_status->get_wheel_radius();
    wheel_ppr=xq_status->get_wheel_ppr();
    geometry_msgs::Twist  carTwist = xq_status->get_CarTwist();

    double vx_temp,vtheta_temp;
    vx_temp=command.linear.x;
    vtheta_temp=command.angular.z;
    if(std::fabs(vx_temp)<0.11)
    {
      if(vtheta_temp>0.02&&vtheta_temp<0.3) vtheta_temp=0.3;
      if(vtheta_temp<-0.02&&vtheta_temp>-0.3) vtheta_temp=-0.3;
    }
    if(vx_temp>0 && vx_temp<0.1) vx_temp=0.1;
    if(vx_temp<0 && vx_temp>-0.1) vx_temp=-0.1;
    //转换速度单位，由米转换成转
    speed_lin=command.linear.x/(2.0*PI*radius);
    //speed_ang=command.angular.z*separation/(2.0*PI*radius);
    speed_ang=vtheta_temp*separation/(2.0*PI*radius);

    if(stopFlag_)
    {
      speed_lin=0.0;
      speed_ang=0.0;
    }

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

  //std::cout<<" "<<speed_temp[0]<<" " << speed_temp[1] <<  " "<< command.linear.x <<" "<< command.angular.z <<  " "<< carTwist.linear.x <<" "<< carTwist.angular.z <<std::endl;
  //std::cout<<"radius "<<radius<<std::endl;
  //std::cout<<"ppr "<<wheel_ppr<<std::endl;
  //std::cout<<"pwm "<<speed_temp[0]<<std::endl;
  //  command.linear.x/
    for(i=0;i<2;i++)
    {
     speed[i]=-(int8_t)speed_temp[i];
     speed_debug[i]=speed_temp[i];
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

    // std::cout<<"hbz1 "<<xq_status->car_status.hbz1<<std::endl;
    // std::cout<<"hbz2 "<<xq_status->car_status.hbz2<<std::endl;
    // std::cout<<"hbz3 "<<xq_status->car_status.hbz3<<std::endl;
    // if(xq_status->get_status()==2)
    // {
    //   //有障碍物
    //   if(xq_status->car_status.hbz1<30&&xq_status->car_status.hbz1>0&&cmd_str[6]==(char)0x46)
    //   {
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz2<30&&xq_status->car_status.hbz2>0&&cmd_str[5]==(char)0x46)
    //   {
    //     cmd_str[5]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz3<20&&xq_status->car_status.hbz3>0&&(cmd_str[5]==(char)0x42||cmd_str[6]==(char)0x42))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz1<15&&xq_status->car_status.hbz1>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz2<15&&xq_status->car_status.hbz2>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    // }

    boost::mutex::scoped_lock lock(mMutex);

    if(!MoveFlag)
    {
      cmd_str[5]=(char)0x53;
      cmd_str[6]=(char)0x53;
    }
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,13);
    }
    last_ordertime=ros::WallTime::now();
   // command.linear.x
}

bool DiffDriverController::checkStop()
{

    static time_t t1=time(NULL),t2;
    int i=0;
    double speed_lin=0,speed_ang=0,speed_temp[2];
    char speed[2]={0,0};//右一左二
    char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};

    if(xq_status->get_status()==0) return false;//底层还在初始化

    if(stopFlag_)
    {
      speed_lin=0.0;
      speed_ang=0.0;
    }
    else
    {
      if((xq_status->car_status.hbz1+xq_status->car_status.hbz2+xq_status->car_status.hbz4)>0.1&&(xq_status->car_status.hbz1+xq_status->car_status.hbz2+xq_status->car_status.hbz4)<4.0) return true;
      return false;
    }
    float scale=1.0;

    //转出最大速度百分比,并进行限幅
    speed_temp[1]=scale*(speed_lin+speed_ang/2)/max_wheelspeed*100.0;
    speed_temp[1]=std::min(speed_temp[1],100.0);
    speed_temp[1]=std::max(-100.0,speed_temp[1]);

    speed_temp[0]=scale*(speed_lin-speed_ang/2)/max_wheelspeed*100.0;
    speed_temp[0]=std::min(speed_temp[0],100.0);
    speed_temp[0]=std::max(-100.0,speed_temp[0]);

  //std::cout<<" "<<speed_temp[0]<<" " << speed_temp[1] <<  " "<< command.linear.x <<" "<< command.angular.z <<  " "<< carTwist.linear.x <<" "<< carTwist.angular.z <<std::endl;
  //std::cout<<"radius "<<radius<<std::endl;
  //std::cout<<"ppr "<<wheel_ppr<<std::endl;
  //std::cout<<"pwm "<<speed_temp[0]<<std::endl;
  //  command.linear.x/
    for(i=0;i<2;i++)
    {
     speed[i]=-(int8_t)speed_temp[i];
     speed_debug[i]=speed_temp[i];
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

    // std::cout<<"hbz1 "<<xq_status->car_status.hbz1<<std::endl;
    // std::cout<<"hbz2 "<<xq_status->car_status.hbz2<<std::endl;
    // std::cout<<"hbz3 "<<xq_status->car_status.hbz3<<std::endl;
    // if(xq_status->get_status()==2)
    // {
    //   //有障碍物
    //   if(xq_status->car_status.hbz1<30&&xq_status->car_status.hbz1>0&&cmd_str[6]==(char)0x46)
    //   {
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz2<30&&xq_status->car_status.hbz2>0&&cmd_str[5]==(char)0x46)
    //   {
    //     cmd_str[5]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz3<20&&xq_status->car_status.hbz3>0&&(cmd_str[5]==(char)0x42||cmd_str[6]==(char)0x42))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz1<15&&xq_status->car_status.hbz1>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz2<15&&xq_status->car_status.hbz2>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    // }

    boost::mutex::scoped_lock lock(mMutex);

    if(!MoveFlag)
    {
      cmd_str[5]=(char)0x53;
      cmd_str[6]=(char)0x53;
    }
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,13);
    }
    last_ordertime=ros::WallTime::now();
    return true;
   // command.linear.x
}

void DiffDriverController::UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status)
{
    boost::mutex::scoped_lock lock(mStausMutex_);
    galileoStatus_.nav_status = current_receive_status.navStatus;
    galileoStatus_.visual_status = current_receive_status.visualStatus;
    galileoStatus_.charge_status = current_receive_status.chargeStatus;
    galileoStatus_.power = current_receive_status.power;
    galileoStatus_.target_numID = current_receive_status.targetNumID;
    galileoStatus_.target_status = current_receive_status.targetStatus;
    galileoStatus_.target_distance = current_receive_status.targetDistance;
    galileoStatus_.angle_goal_status = current_receive_status.angleGoalStatus;
    galileoStatus_.control_speed_x = current_receive_status.controlSpeedX;
    galileoStatus_.control_speed_theta = current_receive_status.controlSpeedTheta;
    galileoStatus_.current_speed_x = current_receive_status.currentSpeedX;
    galileoStatus_.current_speed_theta = current_receive_status.currentSpeedTheta;
}
bool DiffDriverController::dealBackSwitch()
{
  boost::mutex::scoped_lock lock(mStausMutex_);
  if(galileoStatus_.nav_status ==1 )
  {
    if(galileoStatus_.visual_status != 0)
    {
      if(galileoStatus_.target_numID != 0)
      {
        if(galileoStatus_.target_status == 0)
        {
          //判断开关是否按下
          if(xq_status->car_status.hbz3==1)
          {
            back_touch_falg_ = true;
            last_touchtime_ = ros::WallTime::now();
          }
          else
          {
            //消除按键抖动
            ros::WallDuration t_diff = ros::WallTime::now() - last_touchtime_;
            if(back_touch_falg_ && t_diff.toSec()>=0.1)
            {
              //开关松开
              back_touch_falg_ = false;
              //发布会厨房命令
              galileo_serial_server::GalileoNativeCmds currentCmds;
              currentCmds.header.stamp = ros::Time::now();
              currentCmds.header.frame_id = "xq_serial_server";
              currentCmds.length = 2;
              currentCmds.data.resize(2);
              currentCmds.data[0] = (char)0x67;
              currentCmds.data[1] = (char)0x00;
              mgalileoCmdsPub_.publish(currentCmds);
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}




















}
