#include "DiffDriverController.h"
#include <time.h>

namespace xqserial_server
{


  DiffDriverController::DiffDriverController()
  {
    max_wheelspeed=200;
    cmd_topic="cmd_vel";
    xq_status=new StatusPublisher();
    cmd_serial=NULL;
    MoveFlag=true;
    last_ordertime=ros::WallTime::now();
    DetectFlag_=true;
    galileoStatus_.mapStatus = 0;
    R_min_ = 0.25;
    shutdown_flag_ = false;
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
    galileoStatus_.mapStatus = 0;
    R_min_ = r_min;
    mgalileoCmdsPub_ = mNH_.advertise<galileo_serial_server::GalileoNativeCmds>("/galileo/cmds", 0, true);
    back_touch_falg_ = false;
    last_touchtime_ = ros::WallTime::now();
    fastStopFlag_ = false;

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
    acc_vx_set_ = 0.4; //1.0
    acc_wz_set_ = 1.7; //6.0

    angle_limit_ = 1.6;
    tran_dist_ = 0.5;
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

    scan_min_dist_ = x_limit_*2;
    move_forward_flag_ = true;
    shutdown_flag_ = false;
  }

  void DiffDriverController::run()
  {
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/global_move_flag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    ros::Subscriber sub5 = nodeHandler.subscribe("/move_base/StatusFlag", 1, &DiffDriverController::updateFastStopFlag,this);
    ros::Subscriber sub6 = nodeHandler.subscribe("/galileo/status", 1, &DiffDriverController::UpdateNavStatus, this);
    ros::Subscriber sub7 = nodeHandler.subscribe("/scan", 1, &DiffDriverController::updateScan, this);
    ros::ServiceServer service = nodeHandler.advertiseService("shutdown", &DiffDriverController::UpdateC4Flag, this);
    ros::spin();
  }

  void DiffDriverController::Refresh()
  {
    ros::WallDuration t_diff = ros::WallTime::now() - last_ordertime;
    if(t_diff.toSec()<6.0 && (xq_status->car_status.hbz_status & 0x04)==0 && !shutdown_flag_)
    {
      //ROS_ERROR("oups1");
      if(t_diff.toSec()>1.0 || xq_status->get_status()<=0)
      {
        //ROS_ERROR("oups2 %f %d %d",t_diff.toSec(),xq_status->get_status(),xq_status->car_status.hbz3);
        boost::mutex::scoped_lock lock(mMutex);
        //命令超时1秒，或者imu还在初始化
        linear_x_goal_ = 0;
        theta_z_goal_ = 0;
        R_goal_ = 0;

        acc_vx_ = acc_vx_set_;
        acc_wz_ = acc_wz_set_;
      }
      //ROS_ERROR("oups3 %f %f, %f %f",linear_x_goal_,theta_z_goal_, linear_x_current_, theta_z_current_);
      UpdateSpeed();
      //geometry_msgs::Twist car_twist_now =  xq_status->get_CarTwist();
      //ROS_ERROR("oups4 %f %f, %f %f, %f",linear_x_goal_,theta_z_goal_, linear_x_current_, theta_z_current_,car_twist_now.linear.x);
      send_speed();
    }
    else
    {
      //超时或者c3按下了
      boost::mutex::scoped_lock lock(mScanMutex_);
      scan_min_dist_ = x_limit_*2;
      move_forward_flag_ = true;
      send_release();
    }
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

  void DiffDriverController::send_release()
  {
    boost::mutex::scoped_lock lock(mMutex);
    //下发底层ｉｍｕ标定命令
    char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x51};
    if(NULL!=cmd_serial)
    {
      cmd_serial->write(cmd_str,5);
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

  void DiffDriverController::updateFastStopFlag(const std_msgs::Int32& fastStopmsg)
  {
    boost::mutex::scoped_lock lock(mStausMutex_);
    if(fastStopmsg.data == 2)
    {
      fastStopFlag_ = true;
    }
    else
    {
      fastStopFlag_ = false;
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
      if(galileoStatus_.mapStatus == 1)
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

    double vx_temp,vtheta_temp;
    vx_temp=linear_x_current_;
    vtheta_temp=theta_z_current_;
    if(std::fabs(vx_temp)<0.05)
    {
      if(vtheta_temp>0.0002&&vtheta_temp<0.05) vtheta_temp=0.05;
      if(vtheta_temp<-0.0002&&vtheta_temp>-0.05) vtheta_temp=-0.05;
    }
    //转换速度单位，由米或弧度转换成RPM
    speed_lin=vx_temp/(2.0*PI*radius)*60;
    speed_ang=vtheta_temp*separation/(2.0*PI*radius)*60;

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

    //ROS_ERROR("speed %f %f %f",speed_temp[0],vx_temp,bar_distance);
    for(i=0;i<2;i++)
    {
      speed[i]=(int8_t)speed_temp[i];
      speed_debug[i]=(int8_t)speed_temp[i];
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
    //ROS_ERROR("linear_x_goal_ %f , %f, %f",linear_x_goal_,linear_x_last_,car_twist_now.linear.x);
    float acc_vx_min_temp = acc_vx_set_;

    float bar_distance = xq_status->get_ultrasonic_min_distance();

    if(!DetectFlag_) bar_distance = 4.2;

    if(bar_distance<0.4 && bar_distance>0.05)
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
      acc_vx_min_temp = std::min(acc_vx_max_, (float)(car_twist_now.linear.x*car_twist_now.linear.x/2.0/std::max(bar_distance - tran_dist_,0.05f)));
    }

    //ROS_ERROR("bar_distance %f, goal %f , %f, %f, acc %f",bar_distance, linear_x_goal_,linear_x_last_,car_twist_now.linear.x,acc_vx_min_temp);

    acc_vx_ = std::max(acc_vx_set_,acc_vx_min_temp); //当前需要的加速度
    acc_wz_ = acc_wz_set_;
    //ROS_ERROR("acc_vx_ %f %f %f, %f",acc_vx_,acc_vx_set_,acc_vx_max_, bar_distance);
    //确定是加速还是减速
    if(linear_x_goal_ < linear_x_last_) acc_vx_ = -acc_vx_;
    if(theta_z_goal_ < theta_z_last_) acc_wz_ = -acc_wz_;
    //根据加速度、目标速度，目标半径，控制频率, 当前速度，得到下一时刻控制速度
    const float dt = 0.02; //50hz
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

    //ROS_ERROR("oups1 %f %f, %f %f, %f",linear_x_goal_,theta_z_goal_, linear_x_current_, theta_z_current_,scan_min_dist_);

    //超声波减速
    float bar_distance = xq_status->get_ultrasonic_min_distance();
    if(!DetectFlag_) bar_distance = 4.2;

    if(bar_distance<0.4 && bar_distance>0.05)
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
      vx_temp = std::min(vx_temp,(float)std::sqrt(std::max(bar_distance - tran_dist_,0.0f)*acc_vx_set_*2));
      //ROS_ERROR("speed1.0 %f ,%f %f",bar_distance,vx_temp, linear_x_goal_);
    }

    if ((!MoveFlag || fastStopFlag_) && vx_temp>0.01)
    {
      vx_temp = 0.;
      //vtheta_temp = 0.;
      //ROS_ERROR("speed1.1 ");
    }

    //超声波避障
    if(DetectFlag_)
    {
      bool forward_flag=true,rot_flag=true;
      forward_flag = xq_status->can_movefoward();

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
      if(galileoStatus_.mapStatus == 1 && std::fabs(R_goal_)>0.001)
      {
        theta_z_goal_ = linear_x_goal_/R_goal_; //确保运动半径不变
      }

      if(galileoStatus_.targetStatus!=1)
      {
        fastStopFlag_ = false;
      }
    }
    //ROS_ERROR("speed1.3 %f %f",linear_x_goal_,theta_z_goal_);
  }

  void DiffDriverController::UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status)
  {
    boost::mutex::scoped_lock lock(mStausMutex_);
    galileoStatus_.navStatus = current_receive_status.navStatus;
    galileoStatus_.visualStatus = current_receive_status.visualStatus;
    galileoStatus_.chargeStatus = current_receive_status.chargeStatus;
    galileoStatus_.mapStatus = current_receive_status.mapStatus;
    galileoStatus_.targetStatus = current_receive_status.targetStatus;
    galileoStatus_.targetNumID = current_receive_status.targetNumID;
  }

  bool DiffDriverController::dealBackSwitch()
  {
    boost::mutex::scoped_lock lock(mStausMutex_);
    if(galileoStatus_.navStatus ==1 )
    {
      if(galileoStatus_.visualStatus != 0)
      {
        if(galileoStatus_.targetNumID != 0)
        {
          if(galileoStatus_.targetStatus == 0)
          {
            //判断开关是否按下
            if((xq_status->car_status.hbz_status & 0x01)==1)  //c1
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
        }
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
      //if(std::fabs(range_k - range_k_1)<0.05 && std::fabs(range_k_1 - range_k_2)<0.05)
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
  }




















}
