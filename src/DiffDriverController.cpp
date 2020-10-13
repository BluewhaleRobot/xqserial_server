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
    R_min_ = 0.25;
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
    R_min_ = r_min;
    mgalileoCmdsPub_ = mNH_.advertise<galileo_serial_server::GalileoNativeCmds>("/galileo/cmds", 0, true);
    back_touch_falg_ = false;
    last_touchtime_ = ros::WallTime::now();
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler("~");
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/global_move_flag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    ros::Subscriber sub5 = nodeHandler.subscribe("/move_base/StatusFlag", 1, &DiffDriverController::updateFastStopFlag,this);
    ros::Subscriber sub6 = nodeHandler.subscribe("/galileo/status", 1, &DiffDriverController::UpdateNavStatus, this);
    ros::ServiceServer service = nodeHandler.advertiseService("shutdown", &DiffDriverController::UpdateC4Flag, this);
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

bool DiffDriverController::UpdateC4Flag(ShutdownRequest &req, ShutdownResponse &res)
{
  ROS_WARN_STREAM("Start processing shutdown request");
  if(req.flag)
  {
    boost::mutex::scoped_lock lock(mMutex_c4);
    //下发底层c4开关命令
    ros::param::set("/xqserial_server/params/c4", 1);
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x4b,(char)0x01};
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

void DiffDriverController::sendHeartbag()
{
  //下发底层心跳包
  char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x4c};
  if(NULL!=cmd_serial)
  {
      cmd_serial->write(cmd_str,5);
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

    float x_filter = command.linear.x, z_filter = command.angular.z ;
    {
      //先过滤速度
      boost::mutex::scoped_lock lock2(mStausMutex_);
      if(galileoStatus_.mapStatus == 1)
      {
        if(command.angular.z <-0.001 || command.angular.z>0.001 )
        {
          float R_now =  std::fabs(command.linear.x / command.angular.z);
          if(R_now < R_min_)
          {
            if(command.angular.z>0.001)
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

    double vx_temp,vtheta_temp;
    vx_temp=x_filter;
    vtheta_temp=z_filter;
    if(std::fabs(vx_temp)<0.1)
    {
      if(vtheta_temp>0.0002&&vtheta_temp<0.1) vtheta_temp=0.1;
      if(vtheta_temp<-0.0002&&vtheta_temp>-0.1) vtheta_temp=-0.1;
    }
    //if(vx_temp>0 && vx_temp<0.1) vx_temp=0.1;
    //if(vx_temp<0 && vx_temp>-0.1) vx_temp=-0.1;

    if(std::fabs(xq_status->get_wheel_v_theta()-carTwist.angular.z)>1.0) vtheta_temp=0;
    if((!xq_status->can_movefoward()) && DetectFlag_ && carTwist.linear.x>=0.01)
    {
      if(x_filter>0)
      {
        vx_temp=-2.3;
        //vtheta_temp=0;
      }
      if(x_filter==0)
      {
        vx_temp=0.0;
        //vtheta_temp=0;
      }
    }

    float bar_distance = xq_status->get_ultrasonic_min_distance();
    if(!DetectFlag_) bar_distance = 4.2;

    // if(bar_distance<=1.2)
    // {
    //   vx_temp = std::min(vx_temp,0.5*(bar_distance-0.2));
    // }

    speed_lin=vx_temp/(2.0*PI*radius);
    //speed_ang=command.angular.z*separation/(2.0*PI*radius);
    speed_ang=vtheta_temp;

    //转出最大速度百分比,并进行限幅
    speed_temp[0]=speed_lin/max_wheelspeed*100.0;
    speed_temp[0]=std::min(speed_temp[0],100.0);
    speed_temp[0]=std::max(-100.0,speed_temp[0]);

    speed_temp[1]=speed_ang/3.0*100.0;
    speed_temp[1]=std::min(speed_temp[1],100.0);
    speed_temp[1]=std::max(-100.0,speed_temp[1]);

    //ROS_ERROR("speed %f %f %f",speed_temp[0],vx_temp,bar_distance);
    for(i=0;i<2;i++)
    {
     speed[i]=speed_temp[i];
     speed_debug[i]=speed_temp[i];
     if(speed[i]<0)
     {
         //if(speed[i]>-5) speed[i]=-4;
         cmd_str[5+i]=(char)0x42;//B
         cmd_str[9+i]=-speed[i];
     }
     else if(speed[i]>0)
     {
         //if(speed[i]<5) speed[i]=4;
         cmd_str[5+i]=(char)0x46;//F
         cmd_str[9+i]=speed[i];
     }
     else
     {
         cmd_str[5+i]=(char)0x53;//S
         cmd_str[9+i]=(char)0x00;
     }
    }

    boost::mutex::scoped_lock lock(mMutex);
    cmdTwist_ = command;
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

void DiffDriverController::check_faster_stop()
{
  boost::mutex::scoped_lock lock2(mStausMutex_);
  if(galileoStatus_.targetStatus != 1) fastStopFlag_ = false;
  static bool last_flag=true;
  int i =0;
  geometry_msgs::Twist car_twist = xq_status->get_CarTwist();
  float vx_temp=0,vtheta_temp=0;
  if(std::fabs(xq_status->get_wheel_v_theta()-car_twist.angular.z)>10.0)
  {
    vtheta_temp=0;
  }else
  {
    if( (!DetectFlag_) || xq_status->get_status()==0 || cmdTwist_.linear.x<=-0.001 || (cmdTwist_.linear.x<=0.01 && std::fabs(cmdTwist_.angular.z)>0.01))
    {
      fastStopFlag_ = false;
      return;
    }
    bool current_fag = !fastStopFlag_;
    if(current_fag) current_fag = xq_status->can_movefoward();
    if( current_fag && last_flag)
    {
      last_flag = current_fag;
      return;
    }

    float current_speed = car_twist.linear.x;
    //pid快速制动
    const float k=1.0;
    vx_temp = -2.3;
    if(car_twist.linear.x<=0.01) vx_temp =0.0;

    if(current_fag) vx_temp =0.0;
    last_flag = current_fag;
  }
  //下发速度
  double radius=0,speed_lin=0,speed_ang=0,speed_temp[2];
  char speed[2]={0,0};//右一左二
  char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
  radius=xq_status->get_wheel_radius();

  speed_lin=vx_temp/(2.0*PI*radius);
  //speed_ang=command.angular.z*separation/(2.0*PI*radius);
  speed_ang=vtheta_temp;
  //转出最大速度百分比,并进行限幅
  speed_temp[0]=speed_lin/max_wheelspeed*100.0;
  speed_temp[0]=std::min(speed_temp[0],100.0);
  speed_temp[0]=std::max(-100.0,speed_temp[0]);

  speed_temp[1]=speed_ang/3.0*100.0;
  speed_temp[1]=std::min(speed_temp[1],100.0);
  speed_temp[1]=std::max(-100.0,speed_temp[1]);


  //ROS_ERROR("speed %f",speed_temp[0]);
  for(i=0;i<2;i++)
  {
   speed[i]=speed_temp[i];
   speed_debug[i]=speed_temp[i];
   if(speed[i]<0)
   {
       //if(speed[i]>-5) speed[i]=-4;
       cmd_str[5+i]=(char)0x42;//B
       cmd_str[9+i]=-speed[i];
   }
   else if(speed[i]>0)
   {
       //if(speed[i]<5) speed[i]=4;
       cmd_str[5+i]=(char)0x46;//F
       cmd_str[9+i]=speed[i];
   }
   else
   {
       cmd_str[5+i]=(char)0x53;//S
       cmd_str[9+i]=(char)0x00;
   }
  }

  boost::mutex::scoped_lock lock(mMutex);

  if(NULL!=cmd_serial)
  {
      cmd_serial->write(cmd_str,13);
  }
  last_ordertime=ros::WallTime::now();
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
          if(xq_status->car_status.hbz1==1)
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
              //发布回零号点命令
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

void DiffDriverController::updateC2C4()
{
  boost::mutex::scoped_lock lock(mMutex_c4);
  int c2_value = 0;
  int c4_value = 0;
  ros::param::param<int>("/xqserial_server/params/out1", c2_value, c2_value);
  ros::param::param<int>("/xqserial_server/params/c4", c4_value, c4_value);
  ROS_DEBUG("c2 %d %d , c4 %d %d",c2_value,xq_status->car_status.hbz2, c4_value, xq_status->car_status.hbz4);
  if(xq_status->car_status.hbz2 != c2_value)
  {
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x4b,(char)0x00};
    cmd_str[5] = c2_value;
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);
    }
  }
  if(xq_status->car_status.hbz4 != c4_value)
  {
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x4e,(char)0x00};
    cmd_str[5] = c4_value;
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);
    }
  }
}













}
