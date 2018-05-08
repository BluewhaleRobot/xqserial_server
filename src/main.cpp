
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
    ros::param::param<std::string>("~port_car", port_car, "/dev/ttyUSB0");
    int baud_car;
    ros::param::param<int>("~baud_car", baud_car, 115200);
    cout<<"port_car:"<<port_car<<" baud_car:"<<baud_car<<endl;

    std::string port_imu;
    ros::param::param<std::string>("~port_imu", port_imu, "/dev/ttyUSB1");
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

    bool BarFlag = true;
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
      xq_diffdriver.setBarFlag(BarFlag);
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

      ros::Rate r(50);//发布周期为50hz
      int i=0;
      while (ros::ok())
      {
          xq_diffdriver.send_synergy_parmas_getOder();//下发协同运动参数上传指令
          usleep(1000);//延时1MS，等待数据上传
          xq_diffdriver.send_wheel_ppr_delta_getOrder();  //下发编码器上传指令
          usleep(2000);//延时2MS，等待数据上传
          if(BarFlag)
          {
            //超声波避障处理
            int  v=0,r=0;
            xq_status.filter_speed(xq_status.car_status.synergy_speed_set,xq_status.car_status.synergy_r_set, v,r);
            xq_diffdriver.sendcmd(v,r);//普通减速
            xq_diffdriver.sendcmd(xq_status.isneed_faststop());//紧急减速
          }
          i++;
          if(serial_car.errorStatus() || serial_car.isOpen()==false)
          {
              cerr<<"Error: serial port closed unexpectedly"<<endl;
              break;
          }
          xq_status.Refresh();//定时发布状态
          //if(i%5==0) xq_diffdriver.Refresh();
          r.sleep();
          //cout<<"run"<<endl;
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
