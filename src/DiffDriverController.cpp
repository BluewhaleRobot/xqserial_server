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

    //ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::updateCmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/globalMoveFlag", 1, &DiffDriverController::updateMoveFlag,this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);
    //ros::spin();
    {
      //角度闭环
      ros::Rate r(50);//发布周期为50hz

      while (ros::ok())
      {
        {

          boost::mutex::scoped_lock lock(mMutex);
          ros::WallDuration t_diff = ros::WallTime::now() - last_ordertime;
          if(t_diff.toSec()>1.5 && t_diff.toSec()<1.7)
          {
            //safety security
            //char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
            //serial.write(cmd_str, 13);
            cmdTwist_.linear.x=0.0;
            cmdTwist_.angular.z=0.0;
            //xq_diffdriver.last_ordertime=ros::WallTime::now();
          }
          int i=0;
          if(i%50==0 && DetectFlag_)
          {
            //下发底层红外开启命令
            char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x01};
            cmd_serial->write(cmd_str,6);
          }
          i++;
        }
        sendcmd2();

        r.sleep();
      }
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
  boost::mutex::scoped_lock lock(mMutex);
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
  boost::mutex::scoped_lock lock(mMutex);
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

void DiffDriverController::updateCmd(const geometry_msgs::Twist &command)
{
  boost::mutex::scoped_lock lock(mMutex);
  cmdTwist_=command;
  if(cmdTwist_.angular.z>-0.2 && cmdTwist_.angular.z<0.0 ) cmdTwist_.angular.z=-0.2;
  if(cmdTwist_.linear.x>-0.1 && cmdTwist_.linear.x<0.0 ) cmdTwist_.linear.x=-0.1;
  if(cmdTwist_.angular.z<0.2 && cmdTwist_.angular.z>0.0 ) cmdTwist_.angular.z=0.2;
  if(cmdTwist_.linear.x<0.1 && cmdTwist_.linear.x>0.0 ) cmdTwist_.linear.x=0.1;
  last_ordertime=ros::WallTime::now();
}

void DiffDriverController::sendcmd2(void)
{
  boost::mutex::scoped_lock lock(mMutex);
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

   static float v_set=0,theta_set=0;
  {
    //线速度环
    const float Ts_v=0.02,PID_v[3]={0.4,0.9,0.0};
    float kp_v,ki_v,kd_v;
    static float v_error_k=0.0,v_error_k_1=0.0,v_error_k_2=0.0;

    kp_v=PID_v[0];
    ki_v=kp_v*Ts_v/PID_v[1];
    kd_v=kp_v*PID_v[2]/Ts_v;

    v_error_k = cmdTwist_.linear.x - carTwist.linear.x;

    //if(v_error_k>-0.01 && v_error_k<0.01) v_error_k=0.0;

    float v_delta_temp1,v_delta_temp2,v_delta;
    v_delta_temp1 = v_error_k - v_error_k_1;
    //v_delta_temp2 = v_error_k - 2*v_error_k_1 + v_error_k_2;

    v_delta = kp_v*v_delta_temp1 + ki_v*v_error_k ;//+kd_v*v_delta_temp2;
    v_error_k_1 = v_error_k;
    v_error_k_2 = v_error_k_1;

    v_set +=v_delta ;

    float max_v = 2.0;//max_wheelspeed*radius*3.14*2/(separation/2);

    if(v_set > max_v) v_set = max_v;
    if(v_set < -max_v) v_set = -max_v;

    if(cmdTwist_.linear.x>-0.05 && cmdTwist_.linear.x<0.05 ) v_set=0.0;
    //std::cout<<" "<<v_set << " " << v_delta << " " << v_error_k <<std::endl ;
  }
  {
  //角速度环
  const float Ts=0.02,PID_Theta[3]={4.0,0.9,0.0};
  float kp,ki,kd;
  static float theta_error_k=0.0,theta_error_k_1=0.0,theta_error_k_2=0.0;

  kp=PID_Theta[0];
  ki=kp*Ts/PID_Theta[1];
  kd=kp*PID_Theta[2]/Ts;

  if(carTwist.angular.z>-0.005 && carTwist.angular.z<0.005) carTwist.angular.z=0;

  theta_error_k = cmdTwist_.angular.z - carTwist.angular.z;
  //if(theta_error_k>-0.01 && theta_error_k<0.01) theta_error_k=0.0;

  float theta_delta_temp1,theta_delta_temp2,theta_delta;
  theta_delta_temp1 = theta_error_k - theta_error_k_1;
  //theta_delta_temp2 = theta_error_k - 2*theta_error_k_1 + theta_error_k_2;

  theta_delta = kp*theta_delta_temp1 + ki*theta_error_k ;//+kd*theta_delta_temp2;
  theta_error_k_1 = theta_error_k;
  theta_error_k_2 = theta_error_k_1;

  theta_set +=theta_delta ;

  float max_theta = 4.0;//max_wheelspeed*radius*3.14*2/(separation/2);

  if(theta_set > max_theta) theta_set = max_theta;
  if(theta_set < -max_theta) theta_set = -max_theta;

  if(cmdTwist_.angular.z>-0.05 && cmdTwist_.angular.z<0.05 ) theta_set=0.0;


  }
  //转换速度单位，由米转换成转

  //speed_lin = cmdTwist_.linear.x/(2.0*PI*radius);
  //speed_ang = cmdTwist_.angular.z*(separation/2.0)/(2.0*PI*radius);
  speed_lin = v_set/(2.0*PI*radius);
  speed_ang = theta_set*(separation/2.0)/(2.0*PI*radius);


  float kkk=0.4;
  if(cmdTwist_.angular.z>0.5)
  {
    kkk=1.0/(1+cmdTwist_.angular.z);
  }
  else if(cmdTwist_.angular.z<-0.5)
  {
    kkk=(-cmdTwist_.angular.z)/(1.0-cmdTwist_.angular.z);

  }
  else
  {
    kkk=0.5;
  }


  float scale=std::max(std::abs(speed_lin+kkk*speed_ang),std::abs(speed_lin-(1-kkk)*speed_ang))/max_wheelspeed;
  if(scale>1.0)
  {
    scale=1.0/scale;
  }
  else
  {
    scale=1.0;
  }
  //转出最大速度百分比,并进行限幅
  speed_temp[0]=scale*(speed_lin+kkk*speed_ang)/max_wheelspeed*100.0;
  speed_temp[0]=std::min(speed_temp[0],100.0);
  speed_temp[0]=std::max(-100.0,speed_temp[0]);

  speed_temp[1]=scale*(speed_lin-(1-kkk)*speed_ang)/max_wheelspeed*100.0;
  speed_temp[1]=std::min(speed_temp[1],100.0);
  speed_temp[1]=std::max(-100.0,speed_temp[1]);


  double speed_temp2[2]={0.0,0.0};
  xq_status->get_wheel_speed(speed_temp2);
  //std::cout<<" "<<speed_temp[0]<<" " << speed_temp[1] <<" "<<speed_temp2[0]<<" " << speed_temp2[1] << " " << theta_set << " "  << theta_delta << " "<<cmdTwist_.angular.z << " " << theta_sum/8.0 <<std::endl;
  //std::cout<<" "<<speed_temp[0]<<" " << speed_temp[1] <<" "<<speed_temp2[0]<<" " << speed_temp2[1] << " "<< cmdTwist_.linear.x <<" "<< cmdTwist_.angular.z  <<" " << carTwist.linear.x << " " << carTwist.angular.z <<" " <<std::endl;

//std::cout<<"radius "<<radius<<std::endl;
//std::cout<<"ppr "<<wheel_ppr<<std::endl;
//std::cout<<"pwm "<<speed_temp[0]<<std::endl;
//  command.linear.x/
  for(i=0;i<2;i++)
  {
   speed[i]=speed_temp[i];
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

  // std::cout<<"hbz1 "<<xq_status->car_status.hbz1<<std::endl;
  // std::cout<<"hbz2 "<<xq_status->car_status.hbz2<<std::endl;
  // std::cout<<"hbz3 "<<xq_status->car_status.hbz3<<std::endl;
  // if(xq_status->get_status()==2)
  // {
  //   //有障碍物
  //   if(xq_status->car_status.hbz1<30&&xq_status->car_status.hbz1>0&&cmd_str[6]==(char)0x46)
  //   {
  //     cmd_str[6]=(char)0x53;
  //   }
  //   if(xq_status->car_status.hbz2<30&&xq_status->car_status.hbz2>0&&cmd_str[5]==(char)0x46)
  //   {
  //     cmd_str[5]=(char)0x53;
  //   }
  //   if(xq_status->car_status.hbz3<20&&xq_status->car_status.hbz3>0&&(cmd_str[5]==(char)0x42||cmd_str[6]==(char)0x42))
  //   {
  //     cmd_str[5]=(char)0x53;
  //     cmd_str[6]=(char)0x53;
  //   }
  //   if(xq_status->car_status.hbz1<15&&xq_status->car_status.hbz1>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
  //   {
  //     cmd_str[5]=(char)0x53;
  //     cmd_str[6]=(char)0x53;
  //   }
  //   if(xq_status->car_status.hbz2<15&&xq_status->car_status.hbz2>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
  //   {
  //     cmd_str[5]=(char)0x53;
  //     cmd_str[6]=(char)0x53;
  //   }
  // }
  if(!MoveFlag)
  {
    cmd_str[5]=(char)0x53;
    cmd_str[6]=(char)0x53;
  }
  if(NULL!=cmd_serial)
  {
      cmd_serial->write(cmd_str,13);
  }
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
    double vx_temp,vtheta_temp;
    vx_temp=command.linear.x;
    vtheta_temp=command.angular.z;
    if(std::fabs(vx_temp)<0.11)
    {
      if(vtheta_temp>0.02&&vtheta_temp<0.3) vtheta_temp=0.3;
      if(vtheta_temp<-0.02&&vtheta_temp>-0.3) vtheta_temp=-0.3;
    }
    if(std::fabs(vtheta_temp)<0.1)
    {
      if(vx_temp>0 && vx_temp<0.1 ) vx_temp=0.1;
      if(vx_temp<0 && vx_temp>-0.1) vx_temp=-0.1;
    }

    //转换速度单位，由米转换成转
    speed_lin=command.linear.x/(2.0*PI*radius);

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

  //std::cout<<"radius "<<radius<<std::endl;
  //std::cout<<"ppr "<<wheel_ppr<<std::endl;
  //std::cout<<"pwm "<<speed_temp[0]<<std::endl;
  //  command.linear.x/
    for(i=0;i<2;i++)
    {
     speed[i]=speed_temp[i];
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

    // std::cout<<"hbz1 "<<xq_status->car_status.hbz1<<std::endl;
    // std::cout<<"hbz2 "<<xq_status->car_status.hbz2<<std::endl;
    // std::cout<<"hbz3 "<<xq_status->car_status.hbz3<<std::endl;
    // if(xq_status->get_status()==2)
    // {
    //   //有障碍物
    //   if(xq_status->car_status.hbz1<30&&xq_status->car_status.hbz1>0&&cmd_str[6]==(char)0x46)
    //   {
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz2<30&&xq_status->car_status.hbz2>0&&cmd_str[5]==(char)0x46)
    //   {
    //     cmd_str[5]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz3<20&&xq_status->car_status.hbz3>0&&(cmd_str[5]==(char)0x42||cmd_str[6]==(char)0x42))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz1<15&&xq_status->car_status.hbz1>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    //   if(xq_status->car_status.hbz2<15&&xq_status->car_status.hbz2>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
    //   {
    //     cmd_str[5]=(char)0x53;
    //     cmd_str[6]=(char)0x53;
    //   }
    // }
    boost::mutex::scoped_lock lock(mMutex);
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






















}
