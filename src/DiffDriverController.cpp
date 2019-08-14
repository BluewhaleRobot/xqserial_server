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
    linear_x_ = 0.;
    theta_z_ = 0.;
    galileoStatus_.map_status = 0;
    R_min_ = 0.5;
}

DiffDriverController::DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_,double r_min)
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
    linear_x_ = 0.;
    theta_z_ = 0.;
    galileoStatus_.map_status = 0;
    R_min_ = r_min;
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler("~");
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/globalMoveFlag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    ros::Subscriber sub5 = nodeHandler.subscribe("/move_base/StatusFlag", 1, &DiffDriverController::updateStopFlag,this);
    ros::Subscriber sub6 = nodeHandler.subscribe("/galileo/status", 1, &DiffDriverController::UpdateNavStatus, this);
    ros::ServiceServer service = nodeHandler.advertiseService("shutdown", &DiffDriverController::UpdateC4Flag, this);
    ros::spin();
}

bool DiffDriverController::UpdateC4Flag(ShutdownRequest &req, ShutdownResponse &res)
{
  ROS_WARN_STREAM("Start processing shutdown request");
  if(req.flag)
  {
    //下发底层c4开关命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x4b,(char)0x00};
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);
    }
    ROS_WARN_STREAM("Send shutdown command to driver");
    res.result = true;
  }
  else{
    ROS_WARN_STREAM("Shutdown set to False");
    res.result = false;
  }
  return true;
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

void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
{
    boost::mutex::scoped_lock lock(mMutex);
    linear_x_ = command.linear.x ;
    theta_z_ = command.angular.z;
    last_ordertime=ros::WallTime::now();
    this->filterSpeed();
    this->send_speed();
}

void DiffDriverController::send_speed()
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
  vx_temp=linear_x_;
  vtheta_temp=theta_z_;
  if(std::fabs(vx_temp)<0.11)
  {
    if(vtheta_temp>0.02&&vtheta_temp<0.3) vtheta_temp=0.3;
    if(vtheta_temp<-0.02&&vtheta_temp>-0.3) vtheta_temp=-0.3;
  }
  //转换速度单位，由米转换成转
  speed_lin=vx_temp/(2.0*PI*radius);
  //speed_ang=command.angular.z*separation/(2.0*PI*radius);
  speed_ang=vtheta_temp*separation/(2.0*PI*radius);

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

  for(i=0;i<2;i++)
  {
   speed[i]=(int8_t)speed_temp[i];
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

  if(NULL!=cmd_serial)
  {
      cmd_serial->write(cmd_str,13);
  }
}

bool DiffDriverController::checkStop()
{
    //false 表示已经解除
    //true 表示还没解除
    boost::mutex::scoped_lock lock(mMutex);

    bool return_flag=false;

    this->filterSpeed();

    if(xq_status->get_status()==0) return false;//底层还在初始化

    if(galileoStatus_.target_status != 1)
    {
      return_flag = false;
    }

    if(stopFlag_)
    {
      return_flag = true;
    }
    else
    {
      if((xq_status->car_status.hbz1+xq_status->car_status.hbz2+xq_status->car_status.hbz4)>0.1&&(xq_status->car_status.hbz1+xq_status->car_status.hbz2+xq_status->car_status.hbz4)<4.0) return_flag = true;
      bool forward_flag,rot_flag;
      xq_status->get_canmove_flag(forward_flag,rot_flag);
      if(!forward_flag) return_flag = true;
      return_flag = false;
    }
    this->send_speed();
    return return_flag;
}

void DiffDriverController::filterSpeed()
{
  double vx_temp,vtheta_temp;
  vx_temp = linear_x_;
  vtheta_temp = theta_z_;

  //超声波减速
  // float bar_distance = xq_status->get_ultrasonic_min_distance();
  // if(!BarFlag) bar_distance = 4.2;
  //
  // if(bar_distance<=1.2)
  // {
  //   vx_temp = std::min(vx_temp,0.5*(bar_distance-0.2));
  // }
  if (!MoveFlag || stopFlag_)
  {
    vx_temp = 0.;
    //vtheta_temp = 0.;
  }

  //超声波避障
  if(DetectFlag_)
  {
    bool forward_flag=true,rot_flag=true;
    xq_status->get_canmove_flag(forward_flag,rot_flag);
    if(!forward_flag && vx_temp>0.01)
    {
      vx_temp = 0.;
    }
    if(!rot_flag)
    {
      vtheta_temp = 0.;
    }
  }

  {
    //先过滤速度
    boost::mutex::scoped_lock lock(mStausMutex_);
    if(galileoStatus_.map_status == 1)
    {
      if(vtheta_temp <-0.001 || vtheta_temp>0.001 )
      {
        float R_now =  std::fabs(vx_temp / vtheta_temp);
        if(R_now < R_min_)
        {
          if(vtheta_temp>0.001)
          {
            vtheta_temp = std::fabs(vx_temp/R_min_);
          }
          else
          {
            vtheta_temp = -std::fabs(vx_temp/R_min_);
          }
        }
      }
    }
  }

  linear_x_ = vx_temp;
  theta_z_ = vtheta_temp;
}

void DiffDriverController::UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status)
{
    boost::mutex::scoped_lock lock(mStausMutex_);
    galileoStatus_.nav_status = current_receive_status.navStatus;
    galileoStatus_.visual_status = current_receive_status.visualStatus;
    galileoStatus_.charge_status = current_receive_status.chargeStatus;
    galileoStatus_.map_status = current_receive_status.mapStatus;
    galileoStatus_.power = current_receive_status.power;
    galileoStatus_.target_numID = current_receive_status.targetNumID;
    galileoStatus_.target_status = current_receive_status.targetStatus;
    galileoStatus_.target_distance = current_receive_status.targetDistance;
    galileoStatus_.angle_goal_status = current_receive_status.angleGoalStatus;
    galileoStatus_.control_speed_x = current_receive_status.controlSpeedX;
    galileoStatus_.control_speed_theta = current_receive_status.controlSpeedTheta;
    galileoStatus_.current_speed_x = current_receive_status.currentSpeedX;
    galileoStatus_.current_speed_theta = current_receive_status.currentSpeedTheta;

    if(galileoStatus_.target_status != 1)
    {
      stopFlag_ = false;
    }
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
