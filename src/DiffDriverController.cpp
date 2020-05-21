#include "DiffDriverController.h"
#include <time.h>

namespace xqserial_server
{

DiffDriverController::DiffDriverController()
{
    max_wheelspeed = 2.0;
    cmd_topic = "cmd_vel";
    xq_status = new StatusPublisher();
    cmd_serial = NULL;
    MoveFlag = true;
    galileoStatus_.mapStatus = 0;
    R_min_ = 0.25;
}

DiffDriverController::DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *xq_status_, CallbackAsyncSerial *cmd_serial_,double r_min)
{
    MoveFlag = true;
    max_wheelspeed = max_speed_;
    cmd_topic = cmd_topic_;
    xq_status = xq_status_;
    cmd_serial = cmd_serial_;
    R_min_ = r_min;

    mcalibrate_flag = false;
    mA = cv::Mat::zeros(20, 2, CV_32F);
    mb = cv::Mat::zeros(20, 1, CV_32F);
    mx = cv::Mat::zeros(2, 1, CV_32F);
    mcurrent_step = 0;
    mstep_now = 0;
    mt1_flag = false;
    mt2_flag = false;

}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration, this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/global_move_flag", 1, &DiffDriverController::updateMoveFlag, this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag, this);
    ros::Subscriber sub5 = nodeHandler.subscribe("/galileo/status", 1, &DiffDriverController::UpdateNavStatus, this);
    ros::Subscriber sub6 = nodeHandler.subscribe("/xqserial_server/calibFlag", 1, &DiffDriverController::updateCalibFlag, this);
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
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 5);
        }
    }
}
void DiffDriverController::updateBarDetectFlag(const std_msgs::Bool &DetectFlag)
{
    if (DetectFlag.data)
    {
        //下发底层红外开启命令
        char cmd_str[6] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x44, (char)0x01};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 6);
        }
    }
    else
    {
        //下发底层红外禁用命令
        char cmd_str[6] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x44, (char)0x00};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 6);
        }
    }
}
void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
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
    float x_filter = command.linear.x, z_filter = command.angular.z ;
    {
      //先过滤速度
      boost::mutex::scoped_lock lock(mStausMutex_);
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
    //转换速度单位，由米转换成转
    speed_lin = x_filter / (2.0 * PI * radius);
    speed_ang = z_filter * separation / (2.0 * PI * radius);

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

    //std::cout<<"radius "<<radius<<std::endl;
    //std::cout<<"ppr "<<wheel_ppr<<std::endl;
    //std::cout<<"pwm "<<speed_temp[0]<<std::endl;
    //  command.linear.x/
    for (i = 0; i < 2; i++)
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

    boost::mutex::scoped_lock lock(mMutex);
    if (!MoveFlag)
    {
        cmd_str[5] = (char)0x53;
        cmd_str[6] = (char)0x53;
    }
    if (NULL != cmd_serial)
    {
        cmd_serial->write(cmd_str, 13);
    }

    // command.linear.x
}

void DiffDriverController::UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status)
{
    boost::mutex::scoped_lock lock(mStausMutex_);
    galileoStatus_.navStatus = current_receive_status.navStatus;
    galileoStatus_.visualStatus = current_receive_status.visualStatus;
    galileoStatus_.chargeStatus = current_receive_status.chargeStatus;
    galileoStatus_.mapStatus = current_receive_status.mapStatus;

}

void DiffDriverController::updateCalibFlag(const std_msgs::Bool &calibFlag)
{
  boost::mutex::scoped_lock lock(mMutex_calibrate);
  mcalibrate_flag =  calibFlag.data;
  if(mcalibrate_flag)
  {
    mcurrent_step = 0;
    mstep_now = 0;
    mt1_flag = false;
    mt2_flag = false;
  }
  else
  {
    geometry_msgs::Twist current_vel;
    current_vel.linear.x = 0;
    current_vel.linear.y = 0;
    current_vel.linear.z = 0;
    current_vel.angular.x = 0;
    current_vel.angular.y = 0;
    current_vel.angular.z = 0;
    sendcmd(current_vel);
  }
}

void DiffDriverController::dealCalibrateS()
{
  boost::mutex::scoped_lock lock(mMutex_calibrate);
  static float sr1=0,sl1=0,theta1=0,t1=0,vtheta1=0;
  static float sr2=0,sl2=0,theta2=0,t2=0,vtheta2=0;
  static float sr3=0,sl3=0,theta3=0,t3=0,vtheta3=0;
  static float delta_theta21 = 0;
  if(mcalibrate_flag)
  {
    float angular_z = 0;
    if(mcurrent_step <=9)
    {
      angular_z = -1.0 + 0.1*mcurrent_step;
    }
    else
    {
      angular_z = -0.9 + 0.1*mcurrent_step;
    }

    if(mcurrent_step>19)
    {
      //结束标定
      linearSolve(mA, mb, mx);
      float b = xq_status->get_wheel_separation();
      float r = xq_status->get_wheel_radius();
      ROS_ERROR("theory: k1  %f k2 %f ", r/b, 0.0);
      ROS_ERROR("cal: k1  %f k2 %f ", mx.at<float>(0), mx.at<float>(1));
      mcalibrate_flag = false;
    }
    else
    {
      float run_time = 20;
      switch ( mstep_now)
      {
        case 0:
        {
          //原地旋转run_angle角度
            if(!mt1_flag)
            {
              xq_status->get_current_counters(sr1, sl1, theta1, t1, vtheta1);
              ROS_DEBUG("mt1 current_step: %d , sr %f sl %f t %f theta %f",mcurrent_step, sr1, sl1, t1, theta1);
              if(std::fabs(vtheta1)>0.05)
              {
                mt1_flag = true;
                delta_theta21 = 0;
                theta3 = theta1;
              }
            }
            else
            {
              xq_status->get_current_counters(sr2, sl2, theta2, t2, vtheta2);
              ROS_DEBUG("mt2 current_step: %d , sr %f sl %f t %f theta %f",mcurrent_step, sr2, sl2, t2, theta2);
              float delta_sr = sr2 - sr1;
              float delta_sl = sl2 - sl1;
              float delta_theta = theta2 - theta3;
              theta3 = theta2;
              float delta_t21 = t2 -t1;

              if(angular_z<0.01)
              {
                //反转
                if(delta_theta>1.0)
                {
                  delta_theta -= 2*3.1415926;
                }
              }
              else
              {
                //正转
                if(delta_theta<-1.0)
                {
                  delta_theta += 2*3.1415926;
                }
              }

              delta_theta21 += delta_theta;

              if(std::fabs(delta_t21)>run_time)
              {
                mt2_flag = true;
                mstep_now = 1;
                //保存数据
                 mA.at<float>(mcurrent_step,0) = delta_sr - delta_sl;
                 mA.at<float>(mcurrent_step,1) = delta_t21;
                 mb.at<float>(mcurrent_step) = delta_theta21;
                 ROS_ERROR("current_step: %d , sr %f sl %f t %f theta %f",mcurrent_step, delta_sr, delta_sl, delta_t21, delta_theta21);
              }
            }
            //下发角速度
            geometry_msgs::Twist current_vel;
            current_vel.linear.x = 0;
            current_vel.linear.y = 0;
            current_vel.linear.z = 0;
            current_vel.angular.x = 0;
            current_vel.angular.y = 0;
            current_vel.angular.z = angular_z;
            sendcmd(current_vel);
            break;
        }
        case 1:
        {
          //原地等待10秒
            xq_status->get_current_counters(sr3, sl3, theta3, t3, vtheta3);
            float delta_t32 = t3 -t2;
            if(std::fabs(delta_t32)>10.0)
            {
              mstep_now = 2;
            }
            //下发角速度
            geometry_msgs::Twist current_vel;
            current_vel.linear.x = 0;
            current_vel.linear.y = 0;
            current_vel.linear.z = 0;
            current_vel.angular.x = 0;
            current_vel.angular.y = 0;
            current_vel.angular.z = 0;
            sendcmd(current_vel);
            break;
        }
        case 2:
        {
          //继续下一个角速度采集
          mcurrent_step += 1;
          mstep_now = 0;
          mt1_flag = false;
          mt2_flag = false;
          break;
        }
      }
    }
  }
}

void DiffDriverController::linearSolve(const cv::Mat & A, const cv::Mat & b, cv::Mat & x)
{
  // std::stringstream buf_A;
  // buf_A <<"a "<< A<<std::endl;
  // ROS_ERROR("%s",buf_A.str().c_str());
  // std::stringstream buf_b;
  // buf_b <<"b "<< b<<std::endl;
  // ROS_ERROR("%s",buf_b.str().c_str());
  x = (A.t()*A).inv()*(A.t()*b);
  // std::stringstream buf_x;
  // buf_x <<"x "<< x<<std::endl;
  // ROS_ERROR("%s",buf_x.str().c_str());
}

} // namespace xqserial_server
