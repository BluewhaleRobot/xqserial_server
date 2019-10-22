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

    double vx_temp,vtheta_temp;
    vx_temp=command.linear.x;
    vtheta_temp=command.angular.z;
    if(std::fabs(vx_temp)<0.1)
    {
      if(vtheta_temp>0.0002&&vtheta_temp<0.1) vtheta_temp=0.1;
      if(vtheta_temp<-0.0002&&vtheta_temp>-0.1) vtheta_temp=-0.1;
    }
    //if(vx_temp>0 && vx_temp<0.1) vx_temp=0.1;
    //if(vx_temp<0 && vx_temp>-0.1) vx_temp=-0.1;

    if(std::fabs(xq_status->get_wheel_v_theta()-carTwist.angular.z)>1.0) vtheta_temp=0;
    if((!xq_status->can_movefoward()) && DetectFlag_)
    {
      if(command.linear.x>0)
      {
        vx_temp=-2.3;
        vtheta_temp=0;
      }
      if(command.linear.x==0)
      {
        vx_temp=0.0;
        vtheta_temp=0;
      }
    }

    float bar_distance = xq_status->get_ultrasonic_min_distance();
    if(!DetectFlag_) bar_distance = 4.2;

    if(bar_distance<=1.2)
    {
      vx_temp = std::min(vx_temp,0.5*(bar_distance-0.2));
    }

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
     speed[i]=(int8_t)speed_temp[i];
     speed_debug[i]=(int8_t)speed_temp[i];
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
  static bool last_flag=true;
  int i =0;
  geometry_msgs::Twist car_twist = xq_status->get_CarTwist();
  float vx_temp=0,vtheta_temp=0;
  if(std::fabs(xq_status->get_wheel_v_theta()-car_twist.angular.z)>1.5)
  {
    //ROS_ERROR("imu slip error: %f %f",xq_status->get_wheel_v_theta(),car_twist.angular.z);
    vtheta_temp=0;
  }else
  {
    if( (!DetectFlag_) || xq_status->get_status()==0 || cmdTwist_.linear.x<=-0.001)
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




















}
