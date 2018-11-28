
#include "AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"
#include <ros/console.h>

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to xiaoqiang serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "xqserial_server");
    ros::start();


    //获取串口参数
    std::string port_car;
    ros::param::param<std::string>("~port_car", port_car, "/dev/stm32Car");
    int baud_car;
    ros::param::param<int>("~baud_car", baud_car, 57600);
    cout<<"port_car:"<<port_car<<" baud_car:"<<baud_car<<endl;

    std::string port_imu;
    ros::param::param<std::string>("~port_imu", port_imu, "/dev/stm32Imu");
    int baud_imu;
    ros::param::param<int>("~baud_imu", baud_imu, 115200);
    cout<<"port_imu:"<<port_imu<<" baud_car:"<<baud_imu<<endl;

    //获取小车机械参数
    double separation=0,radius=0;
    bool DebugFlag = false;

    ros::param::param<double>("~wheel_separation", separation, 0.33);
    ros::param::param<double>("~wheel_radius", radius, 0.07);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    xqserial_server::StatusPublisher xq_status(separation,radius,DebugFlag);

    bool BarFlag = false;
    ros::param::param<bool>("~bar_flag", BarFlag, true);

    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 4.4);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    try {
      CallbackAsyncSerial serial_car(port_car,baud_car);
      serial_car.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_car,&xq_status,_1,_2));

      CallbackAsyncSerial serial_imu(port_imu,baud_imu);
      serial_imu.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_imu,&xq_status,_1,_2));

      xqserial_server::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial_car,&serial_imu);
      boost::thread cmd2serialThread(& xqserial_server::DiffDriverController::run,&xq_diffdriver);

      // send test flag
      char debugFlagCmd[] = {(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01, 'T'};
      if(DebugFlag){
        std::cout << "Debug mode Enabled" << std::endl;
        //serial_car.write(debugFlagCmd, 5);
        serial_imu.write(debugFlagCmd, 5);
      }
      // send reset cmd
      char resetCmd[] = {(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01, 'I'};
      //serial_car.write(resetCmd, 5);
      serial_imu.write(resetCmd, 5);
      ros::Duration(5).sleep();

      ros::Rate r(100);//发布周期为50hz
      int i=0;
      const char left_speed_mode_cmd1[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x19,(char)0x00,(char)0x2f,(char)0x19,(char)0xd1}; //模式
      const char left_speed_mode_cmd2[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x13,(char)0x64,(char)0x64,(char)0x53,(char)0x24};//加减速度
      const char left_speed_mode_cmd3[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x11,(char)0x00,(char)0x00,(char)0xd9,(char)0xcf};//目标速度
      const char left_speed_mode_cmd4[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x10,(char)0x00,(char)0x1f,(char)0xc9,(char)0xc7};//使能电机 锁轴
      const char left_speed_mode_cmd5[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x10,(char)0x00,(char)0x0f,(char)0xc8,(char)0x0b};//松轴
      const char left_speed_mode_cmd6[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x15,(char)0x00,(char)0x7f,(char)0xd9,(char)0xee};//清除故障

      const char right_speed_mode_cmd1[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x19,(char)0x00,(char)0x2f,(char)0x19,(char)0xe2}; //模式
      const char right_speed_mode_cmd2[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x13,(char)0x64,(char)0x64,(char)0x53,(char)0x17};//加减速度
      const char right_speed_mode_cmd3[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x11,(char)0x00,(char)0x00,(char)0xd9,(char)0xfc};//目标速度
      const char right_speed_mode_cmd4[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x10,(char)0x00,(char)0x1f,(char)0xc9,(char)0xf4};//使能电机 锁轴
      const char right_speed_mode_cmd5[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x10,(char)0x00,(char)0x0f,(char)0xc8,(char)0x38};//松轴
      const char right_speed_mode_cmd6[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x15,(char)0x00,(char)0x7f,(char)0xd9,(char)0xdd};//清除故障

      const char left_query1[8] = {(char)0x01,(char)0x03,(char)0x00,(char)0xd1,(char)0x00,(char)0x02,(char)0x94,(char)0x32};//电压电流
      const char left_query2[8] = {(char)0x01,(char)0x03,(char)0x00,(char)0xd2,(char)0x00,(char)0x02,(char)0x64,(char)0x32};//状态、转速
      const char left_query3[8] = {(char)0x01,(char)0x03,(char)0x00,(char)0xd4,(char)0x00,(char)0x02,(char)0x84,(char)0x33};//编码器计数

      const char right_query1[8] = {(char)0x02,(char)0x03,(char)0x00,(char)0xd1,(char)0x00,(char)0x02,(char)0x94,(char)0x01};//电压电流
      const char right_query2[8] = {(char)0x02,(char)0x03,(char)0x00,(char)0xd2,(char)0x00,(char)0x02,(char)0x64,(char)0x01};//状态、转速
      const char right_query3[8] = {(char)0x02,(char)0x03,(char)0x00,(char)0xd4,(char)0x00,(char)0x02,(char)0x84,(char)0x00};//编码器计数

      while (ros::ok())
      {
          if(serial_car.errorStatus() || serial_car.isOpen()==false)
          {
              cerr<<"Error: serial port closed unexpectedly"<<endl;
              break;
          }

          //先配置速度模式
          if(xq_status.car_status.left_driver_status==0)
          {
            ROS_ERROR("enable left motor!");
            serial_car.write(left_speed_mode_cmd1,8);
            usleep(15000);//延时1MS，等待数据上传
            serial_car.write(left_speed_mode_cmd2,8);
            usleep(15000);//延时1MS，等待数据上传
            serial_car.write(left_speed_mode_cmd3,8);
            usleep(15000);//延时1MS，等待数据上传
            serial_car.write(left_speed_mode_cmd4,8);
            usleep(15000);//延时1MS，等待数据上传
            xq_status.set_register(0x000000d2);
            serial_car.write(left_query2,8);
            usleep(200000);//延时1MS，等待数据上传
            continue;
          }
          if(xq_status.car_status.left_driver_status>1 && xq_status.car_status.left_driver_status<0x80)
          {
            ROS_ERROR("clear left motor error!");
            serial_car.write(left_speed_mode_cmd6,8);
            usleep(15000);//延时1MS，等待数据上传
            xq_status.set_register(0x000000d2);
            serial_car.write(left_query2,8);
            usleep(200000);//延时1MS，等待数据上传
            continue;
          }

          if(xq_status.car_status.right_driver_status==0)
          {
            ROS_ERROR("enable right motor!");
            serial_car.write(right_speed_mode_cmd1,8);
            usleep(15000);//延时1MS，等待数据上传
            serial_car.write(right_speed_mode_cmd2,8);
            usleep(15000);//延时1MS，等待数据上传
            serial_car.write(right_speed_mode_cmd3,8);
            usleep(15000);//延时1MS，等待数据上传
            serial_car.write(right_speed_mode_cmd4,8);
            usleep(15000);//延时1MS，等待数据上传
            xq_status.set_register(0x000000d2);
            serial_car.write(right_query2,8);
            usleep(20000);//延时1MS，等待数据上传
            continue;
          }
          if(xq_status.car_status.right_driver_status>1 && xq_status.car_status.right_driver_status<0x80)
          {
            ROS_ERROR("clear right motor error!");
            serial_car.write(right_speed_mode_cmd6,8);
            usleep(15000);//延时1MS，等待数据上传
            xq_status.set_register(0x000000d2);
            serial_car.write(right_query2,8);
            usleep(20000);//延时1MS，等待数据上传
            continue;
          }
          i++;
          // if(i%50==0)
          // {
          //   boost::thread keep_speed_Thread(&xqserial_server::DiffDriverController::keep_speed,&xq_diffdriver);
          // }
          xq_status.Refresh();//定时发布状态
          r.sleep();
      }
      quit:
      serial_car.close();
      serial_imu.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    ros::shutdown();
    return 0;
}
