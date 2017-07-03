
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
    //ROS_ERROR("Hello %s", "World");

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
    ros::param::param<double>("~wheel_separation", separation, 0.37);
    ros::param::param<double>("~wheel_radius", radius, 0.0625);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    xqserial_server::StatusPublisher xq_status(separation,radius,DebugFlag);

    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 2.0);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    try {
        CallbackAsyncSerial serial_car(port_car,baud_car);
        serial_car.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_car,&xq_status,_1,_2));

        CallbackAsyncSerial serial_imu(port_imu,baud_imu);
        serial_imu.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_imu,&xq_status,_1,_2));

        xqserial_server::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial_car,&serial_imu);
        boost::thread cmd2serialThread(& xqserial_server::DiffDriverController::run,&xq_diffdriver);
        // send test flag
        char debugFlagCmd[] = {0xcd,0xeb,0xd7,0x01, 'T'};
        if(DebugFlag){
          std::cout << "Debug mode Enabled" << std::endl;
          //serial_car.write(debugFlagCmd, 5);
          serial_imu.write(debugFlagCmd, 5);
        }
        // send reset cmd
        char resetCmd[] = {0xcd,0xeb,0xd7,0x01, 'I'};
        //serial_car.write(resetCmd, 5);
        serial_imu.write(resetCmd, 5);

        ros::Rate r(50);//发布周期为50hz
        int i=0;
        while (ros::ok())
        {
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
