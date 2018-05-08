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
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::dealCmd_vel, this);
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

void DiffDriverController::send_wheel_ppr_delta_getOrder()
{
  //下发编码器值上传命令
  boost::mutex::scoped_lock lock(mMutex);
  char cmd_str[20]={(char)0x01,(char)0x40,(char)0x63,(char)0x60,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0xfc,(char)0x02,(char)0x40,(char)0x63,(char)0x60,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0xfb};
  if(NULL!=cmd_serial_car)
  {
      cmd_serial_car->write(cmd_str,20);
  }
}

void DiffDriverController::send_synergy_parmas_getOder()
{
  //下发协同运动参数上传指令
  boost::mutex::scoped_lock lock(mMutex);
  char cmd_str1[10]={(char)0x01,(char)0x40,(char)0x00,(char)0x43,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x7c};
  char cmd_str2[10]={(char)0x01,(char)0x40,(char)0x00,(char)0x43,(char)0x01,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x7b};
  char cmd_str3[10]={(char)0x01,(char)0x40,(char)0x00,(char)0x43,(char)0x02,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x7a};
  char cmd_str4[10]={(char)0x01,(char)0x40,(char)0x02,(char)0x43,(char)0x04,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x76};
  char cmd_str5[10]={(char)0x01,(char)0x40,(char)0x02,(char)0x43,(char)0x10,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x6a};
  char cmd_str6[10]={(char)0x01,(char)0x40,(char)0x10,(char)0x20,(char)0x02,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x8d};
  char cmd_str7[10]={(char)0x01,(char)0x40,(char)0x41,(char)0x60,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x1e};
  char cmd_str8[10]={(char)0x02,(char)0x40,(char)0x41,(char)0x60,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x1d};
  if(NULL!=cmd_serial_car)
  {
      cmd_serial_car->write(cmd_str1,10); //使能
      cmd_serial_car->write(cmd_str2,10); //模式
      cmd_serial_car->write(cmd_str3,10); //控制字
      cmd_serial_car->write(cmd_str4,10); //速度
      cmd_serial_car->write(cmd_str5,10); //半径
      cmd_serial_car->write(cmd_str6,10); //快速停止
      cmd_serial_car->write(cmd_str7,10); //1轴报警
      cmd_serial_car->write(cmd_str8,10); //2轴报警
  }
}

void DiffDriverController::setBarFlag(bool newBarflag)
{
  //避障传感器应急处理标志
  boost::mutex::scoped_lock lock(mMutex);
  BarFlag=newBarflag;
}

void DiffDriverController::sendcmd(bool faster_stop)
{
  //下发紧急停止设置
  boost::mutex::scoped_lock lock(mMutex);
  char cmd_str[10]={(char)0x01,(char)0x2b,(char)0x10,(char)0x20,(char)0x02,(char)0x01,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
  if(faster_stop)
  {
    cmd_str[5]=(char)0x01;
    cmd_str[9]=(char)0xa1;
  }
  else{
    cmd_str[5]=(char)0x00;
    cmd_str[9]=(char)0xa2;
  }
  if(NULL!=cmd_serial_car)
  {
      cmd_serial_car->write(cmd_str,10);
  }
}

void DiffDriverController::sendcmd(const int v, const int r)
{
  boost::mutex::scoped_lock lock(mMutex);
  char cmd_str1[10]={(char)0x01,(char)0x23,(char)0x02,(char)0x43,(char)0x04,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
  char cmd_str2[10]={(char)0x01,(char)0x23,(char)0x02,(char)0x43,(char)0x10,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00};

  char sum=(char)0x00;
  char sum_check;
  char * data;

  //处理v
  data = (char *)&v;
  sum = (char)0x6d;
  cmd_str1[5] = data[0];
  sum += cmd_str1[5];

  cmd_str1[6]=data[1];
  sum += cmd_str1[6];

  cmd_str1[7]=data[2];
  sum += cmd_str1[7];

  cmd_str1[8]=data[3];
  sum += cmd_str1[8];

  sum_check = -sum;
  cmd_str1[9]=sum_check;

  //处理r
  data = (char *)&r;
  sum = (char)0x79;
  cmd_str2[5] = data[0];
  sum += cmd_str2[5];

  cmd_str2[6]=data[1];
  sum += cmd_str2[6];

  cmd_str2[7]=data[2];
  sum += cmd_str2[7];

  cmd_str2[8]=data[3];
  sum += cmd_str2[8];

  sum_check = -sum;
  cmd_str2[9]=sum_check;

  if(NULL!=cmd_serial_car)
  {
      cmd_serial_car->write(cmd_str1,10);
      cmd_serial_car->write(cmd_str2,10);
  }
}

void DiffDriverController::enable_synergy_control()
{
  //设置协同运动模式66
  boost::mutex::scoped_lock lock(mMutex);
  char cmd_str1[10]={(char)0x01,(char)0x2f,(char)0x00,(char)0x43,(char)0x00,(char)0x01,(char)0x00,(char)0x00,(char)0x00,(char)0x8c};
  char cmd_str2[10]={(char)0x01,(char)0x2f,(char)0x00,(char)0x43,(char)0x01,(char)0x66,(char)0x00,(char)0x00,(char)0x00,(char)0x26};
  char cmd_str3[10]={(char)0x01,(char)0x2f,(char)0x00,(char)0x43,(char)0x02,(char)0x86,(char)0x00,(char)0x00,(char)0x00,(char)0x05};
  char cmd_str4[10]={(char)0x01,(char)0x2f,(char)0x00,(char)0x43,(char)0x02,(char)0x0f,(char)0x00,(char)0x00,(char)0x00,(char)0x7c};
  if(NULL!=cmd_serial_car)
  {
      if(xq_status->car_status.mode_enable != 1) cmd_serial_car->write(cmd_str1,10); //使能
      if(xq_status->car_status.mode != 102) cmd_serial_car->write(cmd_str2,10); //模式
      if(xq_status->car_status.driver_error!=0) cmd_serial_car->write(cmd_str3,10); //控制字清除
      if(xq_status->car_status.mode_power_control != 15) cmd_serial_car->write(cmd_str4,10); //控制字使能
  }
}
void DiffDriverController::dealCmd_vel(const geometry_msgs::Twist &command)
{
    float separation = 0,radius = 0,speed_lin = 0,speed_ang = 0;

    if(xq_status->get_status()==0) return;//底层还在初始化
    this->enable_synergy_control();
    //转换成v,r
    separation = xq_status->get_wheel_separation();
    radius = xq_status->get_wheel_radius();

    float speed_lin_max = 2*PI*radius*max_wheelspeed;
    float speed_ang_max = 2*speed_lin_max/separation;
    //最大速度限制
    speed_lin = command.linear.x;
    speed_lin = std::max(std::min(speed_lin,speed_lin_max),-speed_lin_max);
    speed_ang = command.angular.z;
    speed_ang = std::max(std::min(speed_ang,speed_ang_max),-speed_ang_max);
    //开始转换,单位mm
    int v = 0,r = 0;
    if(std::fabs(speed_ang)<0.001)
    {
      //直线运动
      v = (int)(speed_lin*1000.0f);
      r = 0;
    }
    else
    {
      //圆弧运动
      if(std::fabs(speed_lin)<0.001)
      {
        r = 1;
        v = (int)(speed_ang*separation/2.0f*1000.0f);
      }
      else
      {
        r = (int)(speed_lin/speed_ang*1000.0f);
        if(speed_lin>0)
        {
          v = (int)(std::fabs(speed_ang)*(std::fabs(speed_lin/speed_ang)+separation/2.0f)*1000.0f);
        }
        else
        {
          v = (int)(-std::fabs(speed_ang)*(std::fabs(speed_lin/speed_ang)+separation/2.0f)*1000.0f);
        }
      }
    }
    int v_send,r_send;
    if(BarFlag)
    {
      //障碍物预处理
      xq_status->filter_speed(v,r,v_send,r_send);
    }
    else
    {
      v_send = v;
      r_send = r;
    }
    this->sendcmd(v_send, r_send);
}






















}
