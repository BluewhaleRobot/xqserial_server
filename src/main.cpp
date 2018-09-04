
#include "AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to xiaoqiang serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "xqserial_server");
    ros::start();

    //获取串口参数
    std::string port_left;
    ros::param::param<std::string>("~port_left", port_left, "/dev/ttyUSB011");
    int baud_left;
    ros::param::param<int>("~baud_left", baud_left, 115200);
    cout<<"port_left:"<<port_left<<" baud_left:"<<baud_left<<endl;

    std::string port_right;
    ros::param::param<std::string>("~port_right", port_right, "/dev/ttyUSB001");
    int baud_right;
    ros::param::param<int>("~baud_right", baud_right, 115200);
    cout<<"port_right:"<<port_right<<" baud_right:"<<baud_right<<endl;

    //获取小车机械参数
    double separation=0,radius=0;
    bool DebugFlag = false;
    ros::param::param<double>("~wheel_separation", separation, 0.4);
    ros::param::param<double>("~wheel_radius", radius, 0.0825);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    xqserial_server::StatusPublisher xq_status(separation,radius);

    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 5.0);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    try {
      CallbackAsyncSerial serial_left(port_left,baud_left);
      serial_left.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_left,&xq_status,_1,_2));

      CallbackAsyncSerial serial_right(port_right,baud_right);
      serial_right.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_right,&xq_status,_1,_2));

        xqserial_server::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial_left,&serial_right);
        boost::thread cmd2serialThread(& xqserial_server::DiffDriverController::run,&xq_diffdriver);
        // send test flag
        char debugFlagCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'T'};
        if(DebugFlag){
          std::cout << "Debug mode Enabled" << std::endl;
          serial_left.write(debugFlagCmd, 5);
          serial_right.write(debugFlagCmd, 5);
        }
        // send reset cmd
        char resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};
        serial_left.write(resetCmd, 5);
        serial_right.write(resetCmd, 5);

        //下发底层红外开启命令
        char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x01};
        serial_left.write(cmd_str,6);
        serial_right.write(cmd_str,6);

        ros::Rate r(100);//发布周期为50hz
        while (ros::ok())
        {
            static int i=0;
            if(serial_left.errorStatus() || serial_left.isOpen()==false)
            {
                cerr<<"Error: serial_left port closed unexpectedly"<<endl;
                break;
            }
            if(serial_right.errorStatus() || serial_right.isOpen()==false)
            {
                cerr<<"Error: serial_right port closed unexpectedly"<<endl;
                break;
            }
            xq_status.Refresh();//定时发布状态
            ros::WallDuration t_diff = ros::WallTime::now() - xq_diffdriver.last_ordertime;
            if(t_diff.toSec()>1.5 && t_diff.toSec()<1.7)
            {
              //safety security
              // char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};
              // serial.write(cmd_str, 13);
              // std::cout << "oups!" << std::endl;
              //xq_diffdriver.last_ordertime=ros::WallTime::now();
            }
            if(i%100==0 && xq_diffdriver.DetectFlag_)
            {
              //下发底层红外开启命令
              char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x01};
              serial_left.write(cmd_str,6);
              serial_right.write(cmd_str,6);
            }
            i++;
            r.sleep();
            //cout<<"run"<<endl;
        }

        quit:
        serial_left.close();
        serial_right.close();
    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    ros::shutdown();
    return 0;
}
