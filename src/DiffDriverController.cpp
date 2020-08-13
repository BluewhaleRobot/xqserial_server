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

    galileoStatus_.map_status = 0;
    R_min_ = r_min;

    linear_x_current_ = 0;
    theta_z_current_ = 0;

    linear_x_last_ = 0;
    theta_z_last_ = 0;

    linear_x_goal_ = 0;
    theta_z_goal_ = 0;
    R_goal_ = 0;

    acc_vx_max_ = 4.0;
    acc_wz_max_ = 20.0;

    acc_vx_ = 0.4;
    acc_wz_ = 1.7;
    acc_vx_set_ = 0.4;
    acc_wz_set_ = 1.7;

    tran_dist_ = 0.5;

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

void DiffDriverController::Refresh()
{
  ros::WallDuration t_diff = ros::WallTime::now() - last_ordertime;
  if(t_diff.toSec()<6.0)
  {
    //ROS_ERROR("oups1");
    if(t_diff.toSec()>3.0 || xq_status->get_status()<=0 || xq_status->car_status.hbz2 ==1)
    {
      //ROS_ERROR("oups2 %f %d %d",t_diff.toSec(),xq_status->get_status(),xq_status->car_status.hbz2);
      boost::mutex::scoped_lock lock(mMutex);
      //命令超时3秒，或者imu还在初始化
      linear_x_goal_ = 0;
      theta_z_goal_ = 0;
      R_goal_ = 0;

      acc_vx_ = acc_vx_set_;
      acc_wz_ = acc_wz_set_;
    }
    //ROS_ERROR("oups3 %f %f, %f %f",linear_x_goal_,theta_z_goal_, linear_x_current_, theta_z_current_);
    UpdateSpeed();
    //ROS_ERROR("oups4 %f %f, %f %f",linear_x_goal_,theta_z_goal_, linear_x_current_, theta_z_current_);
    send_speed();
  }
  //else
  //{

  //}
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
  //last_ordertime=ros::WallTime::now();
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

void DiffDriverController::sendHeartbag()
{
  //下发底层心跳包
  char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x4c};
  if(NULL!=cmd_serial)
  {
      cmd_serial->write(cmd_str,5);
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
  linear_x_goal_ = command.linear.x ;
  theta_z_goal_ = command.angular.z;
  last_ordertime=ros::WallTime::now();
  if(std::fabs(linear_x_goal_)<=0.01 || std::fabs(theta_z_goal_)<=0.01)
  {
    R_goal_ = 0;
  }
  else
  {
    R_goal_ = linear_x_goal_/theta_z_goal_;
  }
  {
    //建图时过滤转弯半径
    boost::mutex::scoped_lock lock(mStausMutex_);
    if(galileoStatus_.map_status == 1)
    {
      float r_temp = std::max(std::fabs(R_goal_),R_min_);
      if(R_goal_<0) r_temp = -r_temp;
      R_goal_ = r_temp;
      if(std::fabs(theta_z_goal_)>0.01)
      {
        theta_z_goal_ = linear_x_goal_/R_goal_;
      }
      else{
        R_goal_ = 0;
      }
    }
  }
  this->filterGoal();
}

void DiffDriverController::send_speed()
{
  int i=0;
  double separation=0,radius=0,speed_lin=0,speed_ang=0,speed_temp[2];
  char speed[2]={0,0};//右一左二
  char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};

  if(xq_status->get_status()==0) return;//底层还在初始化
  separation=xq_status->get_wheel_separation();
  radius=xq_status->get_wheel_radius();
  
  float max_speed = xq_status->car_status.max_speed;
  if(max_speed < 0.5 || max_speed > 5) max_speed = max_wheelspeed;

  double vx_temp,vtheta_temp;
  vx_temp=linear_x_current_;
  vtheta_temp=theta_z_current_;
  if(std::fabs(vx_temp)<0.11)
  {
    if(vtheta_temp>0.02&&vtheta_temp<0.2) vtheta_temp=0.2;
    if(vtheta_temp<-0.02&&vtheta_temp>-0.2) vtheta_temp=-0.2;
  }
  //转换速度单位，由米转换成转
  speed_lin=vx_temp/(2.0*PI*radius);
  //speed_ang=command.angular.z*separation/(2.0*PI*radius);
  speed_ang=vtheta_temp*separation/(2.0*PI*radius);

  float scale=std::max(std::abs(speed_lin+speed_ang/2.0),std::abs(speed_lin-speed_ang/2.0))/max_speed;
  if(scale>1.0)
  {
    scale=1.0/scale;
  }
  else
  {
    scale=1.0;
  }
  //转出最大速度百分比,并进行限幅
  speed_temp[0]=scale*(speed_lin+speed_ang/2)/max_speed*100.0;
  speed_temp[0]=std::min(speed_temp[0],100.0);
  speed_temp[0]=std::max(-100.0,speed_temp[0]);

  speed_temp[1]=scale*(speed_lin-speed_ang/2)/max_speed*100.0;
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

  boost::mutex::scoped_lock lock(mMutex);
  linear_x_last_ = linear_x_current_;
  theta_z_last_ = theta_z_current_;
}

void DiffDriverController::UpdateSpeed()
{
  //先根据超声波值得到当前最小加速度和最大速度
  boost::mutex::scoped_lock lock(mMutex);
  geometry_msgs::Twist car_twist_now =  xq_status->get_CarTwist();
  this->filterGoal(); //过滤目标速度
  float acc_vx_min_temp = acc_vx_set_;

  float bar_distance = xq_status->get_ultrasonic_min_distance();
  if(!DetectFlag_) bar_distance = 4.2;

  if(bar_distance<=2.2 && bar_distance>0.1 && linear_x_goal_ < linear_x_last_ && linear_x_last_>0)
  {
    //减速过程中，如果速度还是正值，需要确保在障碍物之前减速完成。
    if((bar_distance - tran_dist_)<0.3)
    {
      acc_vx_min_temp = std::min(acc_vx_max_, (float)(2*car_twist_now.linear.x*car_twist_now.linear.x/2.0/std::max(bar_distance - tran_dist_,0.05f)));
    }
    else
    {
      acc_vx_min_temp = std::min(acc_vx_max_, (float)(4*car_twist_now.linear.x*car_twist_now.linear.x/2.0/std::max(bar_distance - tran_dist_,0.05f)));
    }
  }

  acc_vx_ = std::max(acc_vx_set_,acc_vx_min_temp); //当前需要的加速度
  acc_wz_ = acc_wz_set_;

  //确定是加速还是减速
  if(linear_x_goal_ < linear_x_last_) acc_vx_ = -acc_vx_;
  if(theta_z_goal_ < theta_z_last_) acc_wz_ = -acc_wz_;
  //根据加速度、目标速度，目标半径，控制频率, 当前速度，得到下一时刻控制速度
  const float dt = 0.04; //25hz
  float v1 = linear_x_last_ + acc_vx_*dt;

  if(acc_vx_<0)
  {
    linear_x_current_ = std::max(v1, linear_x_goal_);
  }
  else
  {
    linear_x_current_ = std::min(v1,linear_x_goal_);
  }

  float w1 = theta_z_last_ + acc_wz_*dt;
  if(acc_wz_<0)
  {
    theta_z_current_ = std::max(w1, theta_z_goal_);
  }
  else
  {
    theta_z_current_ = std::min(w1,theta_z_goal_);
  }

}

void DiffDriverController::filterGoal()
{
  float vx_temp,vtheta_temp;
  vx_temp = linear_x_goal_;
  vtheta_temp = theta_z_goal_;

  //超声波减速
  float bar_distance = xq_status->get_ultrasonic_min_distance();
  if(!DetectFlag_) bar_distance = 4.2;
  //ROS_ERROR("bar_distance %f", bar_distance);
  if(bar_distance<=2.2 && linear_x_goal_ > 0)
  {
    //负值不用限制,正值不能超过安全刹车距离
    if((bar_distance - tran_dist_)<0.5)
    {
      vx_temp = std::min(vx_temp,(float)std::sqrt(std::max(bar_distance - tran_dist_,0.0f)*0.5*acc_vx_set_*2));
    }
    else
    {
      vx_temp = std::min(vx_temp,(float)std::sqrt(std::max(bar_distance - tran_dist_,0.0f)*0.8*acc_vx_set_*2));
    }
  }

  if ((!MoveFlag || stopFlag_) && vx_temp>0.01)
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

  linear_x_goal_ = vx_temp;

  if(std::fabs(R_goal_)>0.001 && linear_x_goal_>0.1)
  {
    theta_z_goal_ = linear_x_goal_/R_goal_; //确保运动半径不变
  }
  else{
    theta_z_goal_ = vtheta_temp;
  }

  {
    //建图时过滤转弯半径
    boost::mutex::scoped_lock lock(mStausMutex_);
    if(galileoStatus_.map_status == 1 && std::fabs(R_goal_)>0.001)
    {
      theta_z_goal_ = linear_x_goal_/R_goal_; //确保运动半径不变
    }
  }
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
      // http api 不会设置 target id
      // if(galileoStatus_.target_numID != 0)
      // {
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

              // 更新当前WaitReqAction
              http::Request request("http://127.0.0.1:3546/api/v1/action/update_wait_req");
              const http::Response response = request.send("GET");
              if(response.status == 200){
                return true;
              }

              //发布回厨房命令 兼容老版本客户端
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
      // }
    }
  }
  return false;
}




















}
