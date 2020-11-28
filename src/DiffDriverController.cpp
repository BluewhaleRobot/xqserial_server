#include "DiffDriverController.h"
#include <time.h>

namespace xqserial_server
{


DiffDriverController::DiffDriverController()
{
    max_wheelspeed=5.0;
    cmd_topic="cmd_vel";
    xq_status=new StatusPublisher();
    cmd_serial_car=NULL;
    cmd_serial_imu=NULL;
    MoveFlag=true;
    last_ordertime=ros::WallTime::now();
    DetectFlag_=true;
    galileoStatus_.map_status = 0;
    R_min_ = 0.5;
    shutdown_flag_ = false;
}

DiffDriverController::DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_car_,CallbackAsyncSerial* cmd_serial_imu_,double r_min)
{
    MoveFlag=true;
    max_wheelspeed=max_speed_;
    cmd_topic=cmd_topic_;
    xq_status=xq_status_;
    cmd_serial_car=cmd_serial_car_;
    cmd_serial_imu=cmd_serial_imu_;
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

    acc_vx_max_ = 5.0;
    acc_wz_max_ = 21.0;

    acc_vx_ = 0.4;
    acc_wz_ = 1.0;
    acc_vx_set_ = 0.4; //1.0
    acc_wz_set_ = 1.0; //6.0

    angle_limit_ = 1.6;
    tran_dist_ = 0.3;
    x_limit_ = 1.2;
    y_limit_ = 0.3;

    ros::NodeHandle private_nh("~");
    std::string R_laserscan_name;
    if(private_nh.searchParam("R_laserscan", R_laserscan_name))
    {
      private_nh.getParam(R_laserscan_name, R_laserscan_);
    }
    else
    {
      R_laserscan_.push_back(-1.0);
      R_laserscan_.push_back(0.0);

      R_laserscan_.push_back(0.0);
      R_laserscan_.push_back(-1.0);
    }

    std::string T_laserscan_name;
    if(private_nh.searchParam("T_laserscan", T_laserscan_name))
    {
      private_nh.getParam(T_laserscan_name, T_laserscan_);
    }
    else
    {
      T_laserscan_.push_back(0.0);
      T_laserscan_.push_back(0.0);
    }
    last_scantime_ = ros::WallTime::now();
    last_scantime2_ = ros::WallTime::now();

    scan_min_dist_ = x_limit_*2;
    move_forward_flag_ = true;
    shutdown_flag_ = false;
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
    ros::Subscriber sub7 = nodeHandler.subscribe("/scan", 1, &DiffDriverController::updateScan, this);
    ros::ServiceServer service = nodeHandler.advertiseService("shutdown", &DiffDriverController::UpdateC4Flag, this);
    ros::spin();
}

void DiffDriverController::Refresh()
{
  {
    boost::mutex::scoped_lock lock(mScanMutex_);
    ros::WallDuration t_diff = ros::WallTime::now() - last_scantime2_;
    float dt1 = t_diff.toSec();
    if(dt1>2.0)
    {
      move_forward_flag_ = true;
      scan_min_dist_ = 3.0;
    }
  }

  boost::mutex::scoped_lock lock(mShutdownMutex_);
  ros::WallDuration t_diff = ros::WallTime::now() - last_ordertime;
  if(t_diff.toSec()<10.0 && (xq_status->car_status.hbz_status & 0x01)==1 && !shutdown_flag_)
  {
    if(t_diff.toSec()>3.0 || xq_status->get_status()<=0)
    {
      boost::mutex::scoped_lock lock(mMutex);
      //命令超时3秒，或者imu还在初始化
      linear_x_goal_ = 0;
      theta_z_goal_ = 0;
      R_goal_ = 0;

      acc_vx_ = acc_vx_set_;
      acc_wz_ = acc_wz_set_;
    }
    UpdateSpeed();
    send_speed();
  }
  else
  {
    //10秒或C1按下去了
    send_release();
    boost::mutex::scoped_lock lock2(mScanMutex_);
    scan_min_dist_ = x_limit_*2;
    move_forward_flag_ = true;
  }


}

bool DiffDriverController::UpdateC4Flag(ShutdownRequest &req, ShutdownResponse &res)
{
  boost::mutex::scoped_lock lock(mShutdownMutex_);
  ROS_WARN_STREAM("Start processing shutdown request");
  if(req.flag)
  {
    //下发底层c4开关命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x4b,(char)0x00};
    if(NULL!=cmd_serial_imu)
    {
        cmd_serial_imu->write(cmd_str,6);
    }
    this->send_release();
    ROS_WARN_STREAM("Send shutdown command to driver");
    res.result = true;
    shutdown_flag_ = true;
  }
  else{
    ROS_WARN_STREAM("Shutdown set to False");
    res.result = false;
    shutdown_flag_ = false;
  }
  return true;
}


void DiffDriverController::updateStopFlag(const std_msgs::Int32& fastStopmsg)
{
  boost::mutex::scoped_lock lock1(mMutex);
  boost::mutex::scoped_lock lock2(mStausMutex_);
  if(fastStopmsg.data == 2 && galileoStatus_.target_status == 1)
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
    if(NULL!=cmd_serial_imu)
    {
        cmd_serial_imu->write(cmd_str,5);
    }
  }
}

void DiffDriverController::sendHeartbag()
{
  //下发底层心跳包
  char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x4c};
  if(NULL!=cmd_serial_imu)
  {
      cmd_serial_imu->write(cmd_str,5);
  }
}

void DiffDriverController::updateBarDetectFlag(const std_msgs::Bool& DetectFlag)
{
  boost::mutex::scoped_lock lock(mMutex);
  if(DetectFlag.data)
  {
    DetectFlag_=true;
  }
  else
  {
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
      boost::mutex::scoped_lock lock2(mStausMutex_);
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
    //ROS_ERROR("speed3 %f %f, %f",linear_x_goal_,theta_z_goal_,R_goal_);

    //this->filterGoal();
    //ROS_ERROR("speed4 %f %f, %f",linear_x_goal_,theta_z_goal_,R_goal_);
}

void DiffDriverController::send_speed()
{
  int i = 0, wheel_ppr = 100000;
  double separation = 0, radius = 0, speed_lin = 0, speed_ang = 0, speed_temp[2];

  separation = xq_status->get_wheel_separation();
  radius = xq_status->get_wheel_radius();
  wheel_ppr = xq_status->get_wheel_ppr();
  //转换速度单位，由米转换成转
  speed_lin = linear_x_current_ / (2.0 * PI * radius);
  speed_ang = theta_z_current_ * separation / (2.0 * PI * radius);

  float scale = std::max(std::abs(speed_lin + speed_ang / 2.0), std::abs(speed_lin - speed_ang / 2.0)) / max_wheelspeed;
  if (scale > 1.0)
  {
    scale = 1.0 / scale;
  }
  else
  {
    scale = 1.0;
  }

  //进行限幅
  speed_temp[0] = scale * (speed_lin + speed_ang / 2) ;
  speed_temp[0] = std::min(speed_temp[0], 100.0);
  speed_temp[0] = std::max(-100.0, speed_temp[0]);

  speed_temp[1] = scale * (speed_lin - speed_ang / 2) ;
  speed_temp[1] = std::min(speed_temp[1], 100.0);
  speed_temp[1] = std::max(-100.0, speed_temp[1]);

  int32_t left_speed_;
  int32_t right_speed_;

  right_speed_ = (int32_t)(speed_temp[0]*10.0f*wheel_ppr); //单位 0.1 count/S
  left_speed_ = -(int32_t)(speed_temp[1]*10.0f*wheel_ppr); //单位 0.1 count/S

  if(NULL!=cmd_serial_car)
  {
    //下发使能
    const char driver1_enable_cmd[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x50,(char)0x00,(char)0x0f,(char)0xc9,(char)0xdf}; //使能
    const char driver2_enable_cmd[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x50,(char)0x00,(char)0x0f,(char)0xc9,(char)0xec}; //使能
    cmd_serial_car->write(driver1_enable_cmd,8);
    usleep(1000);//延时1MS
    cmd_serial_car->write(driver2_enable_cmd,8);
    usleep(1000);//延时1MS
    //下发速度指令
    //                           0           1          2          3          4          5          6          7          8          9         10         11         12
    char speed1_cmd[13] = {(char)0x01,(char)0x10,(char)0x00,(char)0x40,(char)0x00,(char)0x02,(char)0x04,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
    //                           0           1          2          3          4          5          6          7          8          9         10         11         12
    char speed2_cmd[13] = {(char)0x02,(char)0x10,(char)0x00,(char)0x40,(char)0x00,(char)0x02,(char)0x04,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
    uint8_t crc_hl[2];

    speed1_cmd[7] = (right_speed_>>8)&0xff;
    speed1_cmd[8] = right_speed_&0xff;
    speed1_cmd[9] = (right_speed_>>24)&0xff;
    speed1_cmd[10] = (right_speed_>>16)&0xff;
    xqserial_server::CRC16CheckSum((unsigned char *)speed1_cmd, 11, crc_hl);
    speed1_cmd[11] = crc_hl[0];
    speed1_cmd[12] = crc_hl[1];
    cmd_serial_car->write(speed1_cmd,13);
    usleep(1000);//延时1MS

    speed2_cmd[7] = (left_speed_>>8)&0xff;
    speed2_cmd[8] = left_speed_&0xff;
    speed2_cmd[9] = (left_speed_>>24)&0xff;
    speed2_cmd[10] = (left_speed_>>16)&0xff;
    xqserial_server::CRC16CheckSum((unsigned char *)speed2_cmd, 11, crc_hl);
    speed2_cmd[11] = crc_hl[0];
    speed2_cmd[12] = crc_hl[1];
    cmd_serial_car->write(speed2_cmd,13);
    usleep(1000);//延时1MS
  }

  boost::mutex::scoped_lock lock(mMutex);
  linear_x_last_ = linear_x_current_;
  theta_z_last_ = theta_z_current_;
}

void DiffDriverController::send_release()
{
  boost::mutex::scoped_lock lock(mMutex);
  //下发速度指令
  //                       0           1           2          3          4          5          6          7
  char speed_cmd1[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x50,(char)0x00,(char)0x00,(char)0x89,(char)0xdb};
  char speed_cmd2[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x50,(char)0x00,(char)0x00,(char)0x89,(char)0xe8};
  if(NULL!=cmd_serial_car)
  {
    cmd_serial_car->write(speed_cmd1,8);
    usleep(1000);//延时1MS
    cmd_serial_car->write(speed_cmd2,8);
    usleep(1000);//延时1MS
  }
  linear_x_current_ = 0;
  theta_z_current_ = 0;

  linear_x_last_ = 0;
  theta_z_last_ = 0;

  linear_x_goal_ = 0;
  theta_z_goal_ = 0;
  R_goal_ = 0;

  acc_vx_ = acc_vx_set_;
  acc_wz_ = acc_wz_set_;
}

void DiffDriverController::UpdateSpeed()
{
  //先根据超声波值得到当前最小加速度和最大速度
  boost::mutex::scoped_lock lock(mMutex);

  boost::mutex::scoped_lock lock2(mScanMutex_);

  ros::WallDuration t_diff = ros::WallTime::now() - last_scantime_;
  float dt1 = t_diff.toSec();
  geometry_msgs::Twist car_twist_now =  xq_status->get_CarTwist();

  if(scan_min_dist_<=x_limit_)
  {
    scan_min_dist_ = scan_min_dist_ -  car_twist_now.linear.x * dt1; //利用速度对当前测量距离进行更新
    last_scantime_ = ros::WallTime::now();
  }
  //ROS_ERROR("dt1 %f vx %f scan_dist %f",dt1,car_twist_now.linear.x,scan_min_dist_);
  this->filterGoal(); //过滤目标速度
  float acc_vx_min_temp = acc_vx_set_;

  float bar_distance = xq_status->get_ultrasonic_min_distance();
  if(!DetectFlag_) bar_distance = 4.2;

  if(bar_distance<0.5 && bar_distance>0.05)
  {
    bar_distance = std::min(bar_distance,scan_min_dist_);
  }
  else
  {
    bar_distance = scan_min_dist_;
  }

  if(bar_distance<=2.2 && bar_distance>0.1 && linear_x_goal_ < linear_x_last_ && linear_x_last_>0)
  {
    //减速过程中，如果速度还是正值，需要确保在障碍物之前减速完成。
    acc_vx_min_temp = std::min(acc_vx_max_, (float)(linear_x_last_*linear_x_last_/2.0/std::max(bar_distance-tran_dist_,0.05f)));

    // if((bar_distance - tran_dist_)<0.3)
    // {
    //   acc_vx_min_temp = std::min(acc_vx_max_, (float)(2*car_twist_now.linear.x*car_twist_now.linear.x/2.0/std::max(bar_distance - tran_dist_,0.05f)));
    // }
    // else
    // {
    //   acc_vx_min_temp = std::min(acc_vx_max_, (float)(4*car_twist_now.linear.x*car_twist_now.linear.x/2.0/std::max(bar_distance - tran_dist_,0.05f)));
    // }
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

  if(bar_distance<0.5 && bar_distance>0.05)
  {
    bar_distance = std::min(bar_distance,scan_min_dist_);
  }
  else
  {
    bar_distance = scan_min_dist_;
  }
  //ROS_ERROR("speed1.0.0 %f %f",linear_x_goal_,theta_z_goal_);
  if(bar_distance<=2.2 && linear_x_goal_ > 0)
  {
    //负值不用限制,正值不能超过安全刹车距离
    vx_temp = std::min(vx_temp,(float)std::sqrt(std::max(bar_distance-tran_dist_,0.0f)*acc_vx_set_*2));

    // if((bar_distance - tran_dist_)<0.5)
    // {
    //   vx_temp = std::min(vx_temp,(float)std::sqrt(std::max(bar_distance - tran_dist_,0.0f)*0.5*acc_vx_set_*2));
    // }
    // else
    // {
    //   vx_temp = std::min(vx_temp,(float)std::sqrt(std::max(bar_distance - tran_dist_,0.0f)*0.8*acc_vx_set_*2));
    // }
    //ROS_ERROR("speed1.0 %f ,%f %f",bar_distance,vx_temp, linear_x_goal_);
  }

  if ((!MoveFlag || stopFlag_) && vx_temp>0.01)
  {
    vx_temp = 0.;
    //vtheta_temp = 0.;
    //ROS_ERROR("speed1.1 ");
  }

  //超声波避障
  if(DetectFlag_)
  {
    bool forward_flag=true,rot_flag=true;
    xq_status->get_canmove_flag(forward_flag,rot_flag);

    if(forward_flag) forward_flag = move_forward_flag_;
    if(!forward_flag && vx_temp>0.01)
    {
      vx_temp = 0.;
      //ROS_ERROR("speed1.2");
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
  //ROS_ERROR("speed1.3 %f %f",linear_x_goal_,theta_z_goal_);
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
          //判断c3开关是否按下
          if((xq_status->car_status.hbz_status & 0x04)==1)
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

void DiffDriverController::updateScan(const sensor_msgs::LaserScan& scan_in)
{
  static ros::WallTime last_bartime_ = ros::WallTime::now();
  size_t n_pts = scan_in.ranges.size ();
  Eigen::ArrayXXd ranges (n_pts, 2);
  Eigen::ArrayXXd output (n_pts, 2);

  // Get the ranges into Eigen format
  for (size_t i = 0; i < n_pts; ++i)
  {
    ranges (i, 0) = (double) scan_in.ranges[i];
    ranges (i, 1) = (double) scan_in.ranges[i];
  }

  // Check if our existing co_sine_map is valid
  if (co_sine_map_.rows () != (int)n_pts || angle_min_ != scan_in.angle_min || angle_max_ != scan_in.angle_max )
  {
    ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
    co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
    angle_min_ = scan_in.angle_min;
    angle_max_ = scan_in.angle_max;
    // Spherical->Cartesian projection
    for (size_t i = 0; i < n_pts; ++i)
    {
      co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
      co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
    }
  }

  output = ranges * co_sine_map_;

  boost::mutex::scoped_lock lock(mScanMutex_);
  scan_min_dist_ = x_limit_*2;
  float x1,y1;
  float range_k_2=-2,range_k_1=-1,range_k=0;

  for (size_t i = 0; i < n_pts; ++i)
  {
    range_k = scan_in.ranges[i];
    float range_angle = scan_in.angle_min + i * scan_in.angle_increment;
    if (range_k <= scan_in.range_max && range_k >= scan_in.range_min)
    {
      x1 = R_laserscan_[0]*output(i, 0)+R_laserscan_[1]*output(i,1) + T_laserscan_[0];
      y1 = R_laserscan_[2]*output(i, 0)+R_laserscan_[3]*output(i,1) + T_laserscan_[1];
    }
    //ROS_ERROR("range_k  %f range_angle %f",range_k, range_angle);
    if(std::fabs(range_k - range_k_1)<0.05 && std::fabs(range_k_1 - range_k_2)<0.05)
    {
      //ROS_ERROR("ousp1");
      //3个点之间的距离小于一定值才有效
      if(std::fabs(range_k)<=x_limit_ && std::fabs(range_angle)>=angle_limit_ && std::fabs(range_angle)<=(3.1415926*2 - angle_limit_)) //雷达原始角度范围 0 to 2*pi
      {
        //需要在之前角度和距离内
        //ROS_ERROR("x y angle : %f %f %f",x1,y1,range_angle);
        if(x1<x_limit_ && std::fabs(y1)<y_limit_)
        {
          //点在车前方观测区域
          if(x1<scan_min_dist_)
          {
            scan_min_dist_ = x1;
          }
        }
      }
    }
    range_k_2 = range_k_1;
    range_k_1 = range_k;
  }
  ros::WallDuration bar_time_diff = ros::WallTime::now() - last_bartime_;
  if(scan_min_dist_<=(tran_dist_+0.1) && bar_time_diff.toSec()>0.2)
  {
    //ROS_ERROR("first %f %f", scan_min_dist_,bar_time_diff.toSec());
    scan_min_dist_ = tran_dist_ + 0.1;
  }

  if(scan_min_dist_<=x_limit_)
  {
    last_bartime_ = ros::WallTime::now();
  }
  //ROS_ERROR("scan_min_dist_: %f",scan_min_dist_);
  if(scan_min_dist_<tran_dist_)
  {
    move_forward_flag_ = false;
  }
  else
  {
    move_forward_flag_ = true;
  }
  last_scantime_ = ros::WallTime::now();
  last_scantime2_ = ros::WallTime::now();
}




















}
