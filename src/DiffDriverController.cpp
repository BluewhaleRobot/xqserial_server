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
    stopFlag_ = false;
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
    linear_x_ = 0.;
    theta_z_ = 0.;
    stopFlag_ = false;
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/global_move_flag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    ros::Subscriber sub5 = nodeHandler.subscribe("/move_base/StatusFlag", 1, &DiffDriverController::updateFastStopFlag,this);
    ros::spin();
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
void DiffDriverController::updateFastStopFlag(const std_msgs::Int32& fastStopmsg)
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

    if(stopFlag_)
    {
      return_flag = true;
    }
    else
    {
      bool forward_flag;
      forward_flag = xq_status->can_movefoward();
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
  float bar_distance = xq_status->get_ultrasonic_min_distance();
  if(!DetectFlag_) bar_distance = 4.2;

  if(bar_distance<=1.2)
  {
    vx_temp = std::min(vx_temp,0.5*(bar_distance-0.2));
  }
  if (!MoveFlag || stopFlag_)
  {
    vx_temp = 0.;
    //vtheta_temp = 0.;
  }

  //超声波避障
  if(DetectFlag_)
  {
    bool forward_flag = xq_status->can_movefoward();
    if(!forward_flag && vx_temp>0.01)
    {
      vx_temp = 0.;
    }
    //if(!forward_flag) vtheta_temp = 0.0;
  }

  linear_x_ = vx_temp;
  theta_z_ = vtheta_temp;
}



















}
