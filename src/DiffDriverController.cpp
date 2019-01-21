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
    left_speed_ = 0;
    right_speed_ = 0;
    send_flag_ = false;
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::dealCmd_vel, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/global_move_flag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    ros::Rate r(100);//发布周期为50hz
    int i=0;
    while (ros::ok())
    {
      i++;
      int16_t left_speed,right_speed;
      if(get_speed(left_speed ,right_speed))
      {
        keep_speed();
      }
      else if(i%50==0)
      {
        keep_speed();
      }
      ros::spinOnce();
      r.sleep();
    }
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

void DiffDriverController::dealCmd_vel(const geometry_msgs::Twist &command)
{
  static time_t t1 = time(NULL), t2;
  int i = 0, wheel_ppr = 1;
  double separation = 0, radius = 0, speed_lin = 0, speed_ang = 0, speed_temp[2];
  char speed[2] = {0, 0}; //右一左二
  char cmd_str[13] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x09, (char)0x74, (char)0x53, (char)0x53, (char)0x53, (char)0x53, (char)0x00, (char)0x00, (char)0x00, (char)0x00};

  if (xq_status->get_status() == 0)
    return; //底层还在初始化
  separation = xq_status->get_wheel_separation();
  radius = xq_status->get_wheel_radius();
  wheel_ppr = xq_status->get_wheel_ppr();
  //转换速度单位，由米转换成转
  speed_lin = command.linear.x / (2.0 * PI * radius);
  speed_ang = command.angular.z * separation / (2.0 * PI * radius);

  float scale = std::max(std::abs(speed_lin + speed_ang / 2.0), std::abs(speed_lin - speed_ang / 2.0)) / max_wheelspeed;
  if (scale > 1.0)
  {
    scale = 1.0 / scale;
  }
  else
  {
    scale = 1.0;
  }
  //转出最大速度百分比,并进行限幅
  speed_temp[0] = scale * (speed_lin + speed_ang / 2) / max_wheelspeed * 100.0;
  speed_temp[0] = std::min(speed_temp[0], 100.0);
  speed_temp[0] = std::max(-100.0, speed_temp[0]);

  speed_temp[1] = scale * (speed_lin - speed_ang / 2) / max_wheelspeed * 100.0;
  speed_temp[1] = std::min(speed_temp[1], 100.0);
  speed_temp[1] = std::max(-100.0, speed_temp[1]);

  boost::mutex::scoped_lock lock(mMutex);
  if (!MoveFlag)
  {
    speed_temp[0] = 0;
    speed_temp[1] = 0;
  }
  left_speed_ = (int16_t)(speed_temp[1]*max_wheelspeed*60.0f*8192/3000.0f/100.0f);
  right_speed_ = -(int16_t)(speed_temp[0]*max_wheelspeed*60.0f*8192/3000.0f/100.0f);
  send_flag_ = true;
  //this->send_speed();
}

void DiffDriverController::send_speed()
{
  //boost::mutex::scoped_lock lock(mMutex);
  //下发速度指令
   char speed_cmd[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x11,(char)0x00,(char)0x00,(char)0x94,(char)0x32};//电压电流
  uint8_t crc_hl[2];
  speed_cmd[0] = 0x01;
  speed_cmd[4] = (left_speed_>>8)&0xff;
  speed_cmd[5] = left_speed_&0xff;
  xqserial_server::CRC16CheckSum((unsigned char *)speed_cmd, 6, crc_hl);
  speed_cmd[6] = crc_hl[0];
  speed_cmd[7] = crc_hl[1];
  cmd_serial_car->write(speed_cmd,8);
  //ROS_ERROR("oups50 %x %x %x %x %d %d",speed_cmd[4],speed_cmd[5],left_speed_,right_speed_,left_speed_,right_speed_);
  usleep(15000);//延时1MS，等待数据上传

  speed_cmd[0] = 0x02;
  speed_cmd[4] = (right_speed_>>8)&0xff;
  speed_cmd[5] = right_speed_&0xff;
  xqserial_server::CRC16CheckSum((unsigned char *)speed_cmd, 6, crc_hl);
  speed_cmd[6] = crc_hl[0];
  speed_cmd[7] = crc_hl[1];
  cmd_serial_car->write(speed_cmd,8);
  usleep(15000);//延时1MS，等待数据上传
}

void DiffDriverController::keep_speed()
{
  boost::mutex::scoped_lock lock(mMutex);
  if (xq_status->get_status() == 0) return;
  this->send_speed();
}

bool DiffDriverController::get_speed(int16_t & left_speed ,int16_t & right_speed)
{
  boost::mutex::scoped_lock lock(mMutex);
  left_speed = left_speed_;
  right_speed = right_speed_;
  if(send_flag_)
  {
    send_flag_ = false;
    return true;
  }
  return false;
}




















}
