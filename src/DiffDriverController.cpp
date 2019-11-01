#include "DiffDriverController.h"
#include <time.h>

namespace xqserial_server
{

DiffDriverController::DiffDriverController()
{
    max_wheelspeed = 2.0;
    cmd_topic = "cmd_vel";
    xq_status = new StatusPublisher();
    cmd_serial_left=NULL;
    cmd_serial_right=NULL;
    MoveFlag = true;
    DetectFlag_ = true;
    barFlag_ = false;
    cmd_x_ = 0.0;
    cmd_y_ = 0.0;
    cmd_theta_ = 0.0;
    R_min_ = 0.5;
}

DiffDriverController::DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *xq_status_,CallbackAsyncSerial* cmd_serial_left_,CallbackAsyncSerial* cmd_serial_right_,double r_min)
{
    MoveFlag = true;
    max_wheelspeed = max_speed_;
    cmd_topic = cmd_topic_;
    xq_status = xq_status_;
    cmd_serial_left = cmd_serial_left_;
    cmd_serial_right = cmd_serial_right_;
    cmd_x_ = 0.0;
    cmd_y_ = 0.0;
    cmd_theta_ = 0.0;
    DetectFlag_ = true;
    barFlag_ = false;
    R_min_ = r_min;
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration, this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/globalMoveFlag", 1, &DiffDriverController::updateMoveFlag, this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag, this);
    ros::Subscriber sub5 = nodeHandler.subscribe("/galileo/status", 1, &DiffDriverController::UpdateNavStatus, this);
    ros::spin();
}
void DiffDriverController::updateMoveFlag(const std_msgs::Bool &moveFlag)
{
    boost::mutex::scoped_lock lock(mMutex);
    MoveFlag = moveFlag.data;
}
void DiffDriverController::imuCalibration(const std_msgs::Bool &calFlag)
{
    if (calFlag.data)
    {
        //下发底层ｉｍｕ标定命令
        char cmd_str[5] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, (char)0x43};
        if (NULL != cmd_serial_left)
        {
            cmd_serial_left->write(cmd_str, 5);
        }
        if (NULL != cmd_serial_right)
        {
            cmd_serial_right->write(cmd_str, 5);
        }
    }
}
void DiffDriverController::updateBarDetectFlag(const std_msgs::Bool &DetectFlag)
{
  boost::mutex::scoped_lock lock(mMutex);
  DetectFlag_ = DetectFlag.data;
  // if (DetectFlag.data)
  // {
  //     //下发底层红外开启命令
  //     char cmd_str[6] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x44, (char)0x01};
  //     if (NULL != cmd_serial_right)
  //     {
  //         cmd_serial_right->write(cmd_str, 6);
  //     }
  // }
  // else
  // {
  //     //下发底层红外禁用命令
  //     char cmd_str[6] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x44, (char)0x00};
  //     if (NULL != cmd_serial_right)
  //     {
  //         cmd_serial_right->write(cmd_str, 6);
  //     }
  // }
}
void DiffDriverController::sendcmd()
{
  static time_t t1 = time(NULL), t2;
  int i = 0, wheel_ppr = 1;
  double separation = 0, radius = 0, speed_lin_x = 0,speed_lin_y = 0, speed_ang = 0, speed_temp[4];
  char speed[4] = {0, 0, 0, 0}; //右一左二
  char cmd_str[13] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x09, (char)0x74, (char)0x53, (char)0x53, (char)0x53, (char)0x53, (char)0x00, (char)0x00, (char)0x00, (char)0x00};

  if (xq_status->get_status() == 0)
      return; //底层还在初始化

  separation = xq_status->get_wheel_separation();
  radius = xq_status->get_wheel_radius();
  wheel_ppr = xq_status->get_wheel_ppr();

  float x_filter = cmd_x_, z_filter = cmd_theta_ ;
  {
   //先过滤速度
   boost::mutex::scoped_lock lock(mStausMutex_);
   if(galileoStatus_.mapStatus == 1)
   {
     if(std::fabs(cmd_x_) <-0.001 || std::fabs(cmd_theta_)>0.001 )
     {
       float R_now =  std::fabs(cmd_x_ / cmd_theta_);
       if(R_now < R_min_)
       {
         if(cmd_theta_>0.001)
         {
           z_filter = std::fabs(x_filter/R_min_);
         }
         else
         {
           z_filter = -std::fabs(x_filter/R_min_);
         }
       }
     }
   }
  }
  cmd_x_ = x_filter;
  cmd_theta_ = z_filter;
  //转换速度单位，由米转换成转
  speed_lin_x = cmd_x_ / (2.0 * PI * radius);
  speed_lin_y = cmd_y_ / (2.0 * PI * radius);

  speed_ang = cmd_theta_ * separation/2.0 / (2.0 * PI * radius);

  float motor_w1 = speed_lin_x - speed_lin_y - speed_ang;
  float motor_w2 = speed_lin_x + speed_lin_y + speed_ang;
  float motor_w3 = speed_lin_x + speed_lin_y - speed_ang;
  float motor_w4 = speed_lin_x - speed_lin_y + speed_ang;

  float scale = std::max(std::max(std::abs(motor_w1), std::abs(motor_w2)),std::max(std::abs(motor_w3), std::abs(motor_w4))) / max_wheelspeed;
  if (scale > 1.0)
  {
      scale = 1.0 / scale;
  }
  else
  {
      scale = 1.0;
  }
  //转出最大速度百分比,并进行限幅
  speed_temp[0] = -scale * motor_w1 / max_wheelspeed * 100.0; //反向
  speed_temp[0] = std::min(speed_temp[0], 100.0);
  speed_temp[0] = std::max(-100.0, speed_temp[0]);

  speed_temp[1] = scale * motor_w2 / max_wheelspeed * 100.0;
  speed_temp[1] = std::min(speed_temp[1], 100.0);
  speed_temp[1] = std::max(-100.0, speed_temp[1]);

  speed_temp[2] = scale * motor_w3 / max_wheelspeed * 100.0;
  speed_temp[2] = std::min(speed_temp[2], 100.0);
  speed_temp[2] = std::max(-100.0, speed_temp[2]);

  speed_temp[3] = -scale * motor_w4 / max_wheelspeed * 100.0; //反向
  speed_temp[3] = std::min(speed_temp[3], 100.0);
  speed_temp[3] = std::max(-100.0, speed_temp[3]);

  for (i = 0; i < 4; i++)
  {
      speed[i] = (int8_t)speed_temp[i];
      if (speed[i] < 0)
      {
          cmd_str[5 + i] = (char)0x42; //B
          cmd_str[9 + i] = -speed[i];
      }
      else if (speed[i] > 0)
      {
          cmd_str[5 + i] = (char)0x46; //F
          cmd_str[9 + i] = speed[i];
      }
      else
      {
          cmd_str[5 + i] = (char)0x53; //S
          cmd_str[9 + i] = (char)0x00;
      }
  }

  // std::cout<<"distance1 "<<xq_status->car_status.distance1<<std::endl;
  // std::cout<<"distance2 "<<xq_status->car_status.distance2<<std::endl;
  // std::cout<<"distance3 "<<xq_status->car_status.distance3<<std::endl;
  // if(xq_status->get_status()==2)
  // {
  //   //有障碍物
  //   if(xq_status->car_status.distance1<30&&xq_status->car_status.distance1>0&&cmd_str[6]==(char)0x46)
  //   {
  //     cmd_str[6]=(char)0x53;
  //   }
  //   if(xq_status->car_status.distance2<30&&xq_status->car_status.distance2>0&&cmd_str[5]==(char)0x46)
  //   {
  //     cmd_str[5]=(char)0x53;
  //   }
  //   if(xq_status->car_status.distance3<20&&xq_status->car_status.distance3>0&&(cmd_str[5]==(char)0x42||cmd_str[6]==(char)0x42))
  //   {
  //     cmd_str[5]=(char)0x53;
  //     cmd_str[6]=(char)0x53;
  //   }
  //   if(xq_status->car_status.distance1<15&&xq_status->car_status.distance1>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
  //   {
  //     cmd_str[5]=(char)0x53;
  //     cmd_str[6]=(char)0x53;
  //   }
  //   if(xq_status->car_status.distance2<15&&xq_status->car_status.distance2>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
  //   {
  //     cmd_str[5]=(char)0x53;
  //     cmd_str[6]=(char)0x53;
  //   }
  // }

  char cmd_str_left[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
  char cmd_str_right[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};

  if(!MoveFlag)
  {
    cmd_str[5]=(char)0x53;
    cmd_str[6]=(char)0x53;
  }

  for(i=0;i<13;i++)
  {
    cmd_str_right[i] = cmd_str[i];
    cmd_str_left[i] = cmd_str[i];
  }

  cmd_str_left[5+0] = cmd_str[5+1]; //2号电机
  cmd_str_left[9+0] = cmd_str[9+1]; //2号电机

  cmd_str_left[5+1] = cmd_str[5+2]; //3号电机
  cmd_str_left[9+1] = cmd_str[9+2]; //3号电机


  cmd_str_right[5+0] = cmd_str[5+0]; //1号电机
  cmd_str_right[9+0] = cmd_str[9+0]; //1号电机

  cmd_str_right[5+1] = cmd_str[5+3]; //4号电机
  cmd_str_right[9+1] = cmd_str[9+3]; //4号电机

  if(NULL!=cmd_serial_right)
  {
      cmd_serial_right->write(cmd_str_right,13);
  }

  if(NULL!=cmd_serial_left)
  {
      cmd_serial_left->write(cmd_str_left,13);
  }
}

void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
{
  boost::mutex::scoped_lock lock(mMutex);
  cmd_x_ = command.linear.x;
  cmd_y_ = command.linear.y;
  cmd_theta_ = command.angular.z;
  if(DetectFlag_ && barFlag_)
  {
    if( cmd_x_ > 0) cmd_x_ = 0;
  }
  sendcmd();
    // command.linear.x
}

void DiffDriverController::checkStop()
{
  boost::mutex::scoped_lock lock(mMutex);
  barFlag_ = false;
  if(DetectFlag_)
  {
    if(xq_status->move_avalable_ )
    {
      return;
    }
    barFlag_ = true;
    //发布停止命令
    if(DetectFlag_ && barFlag_)
    {
      if( cmd_x_ > 0) cmd_x_ = 0;
    }
    sendcmd();
  }
}
void DiffDriverController::UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status)
{
    boost::mutex::scoped_lock lock(mStausMutex_);
    galileoStatus_.navStatus = current_receive_status.navStatus;
    galileoStatus_.visualStatus = current_receive_status.visualStatus;
    galileoStatus_.chargeStatus = current_receive_status.chargeStatus;
    galileoStatus_.mapStatus = current_receive_status.mapStatus;

}

} // namespace xqserial_server
