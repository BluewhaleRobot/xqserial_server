
#include "AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"

#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to xiaoqiang serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "xqserial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;
    //获取小车机械参数
    double separation=0,radius=0;
    bool DebugFlag = false;
    ros::param::param<double>("~wheel_separation", separation, 0.37);
    ros::param::param<double>("~wheel_radius", radius, 0.0625);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    xqserial_server::StatusPublisher xq_status(separation,radius);

    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 2.0);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    double kinect_x, kinect_stepsize;
    ros::param::param<double>("~kinect_x", kinect_x, -0.1);
    ros::param::param<double>("~kinect_stepsize", kinect_stepsize, 0.05);

    bool sub_ultrasonic;
    ros::param::param<bool>("~sub_ultrasonic", sub_ultrasonic, false);
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    tf::StampedTransform transform3;
    tf::StampedTransform transform4;
    if(sub_ultrasonic)
    {
      tf::TransformListener listener;
      try{
       listener.waitForTransform("base_link", "sonar1", ros::Time(0), ros::Duration(10) );
       listener.waitForTransform("base_link", "sonar2", ros::Time(0), ros::Duration(10) );
       listener.waitForTransform("base_link", "sonar3", ros::Time(0), ros::Duration(10) );
       listener.waitForTransform("base_link", "sonar4", ros::Time(0), ros::Duration(10) );
       listener.lookupTransform("/base_link", "/sonar1",  ros::Time(0), transform1);
       listener.lookupTransform("/base_link", "/sonar2",  ros::Time(0), transform2);
       listener.lookupTransform("/base_link", "/sonar3",  ros::Time(0), transform3);
       listener.lookupTransform("/base_link", "/sonar4",  ros::Time(0), transform4);

      }
      catch (tf::TransformException &ex) {
       ROS_ERROR("%s",ex.what());
       return false;
      }

    }

    try {
        CallbackAsyncSerial serial(port,baud);
        xqserial_server::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial);
        xq_status.setBarparams(kinect_x,kinect_stepsize,&xq_diffdriver);
        xq_diffdriver.setSonarTf(transform1,transform2,transform3,transform4);
        serial.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update,&xq_status,_1,_2));
        boost::thread cmd2serialThread(& xqserial_server::DiffDriverController::run,&xq_diffdriver);
        // send test flag
        char debugFlagCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'T'};
        if(DebugFlag){
          std::cout << "Debug mode Enabled" << std::endl;
          serial.write(debugFlagCmd, 5);
        }
        // send reset cmd
        char resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};
        serial.write(resetCmd, 5);
        ros::Duration(0.5).sleep();

        ros::Rate r(100);//发布周期为50hz
        int ii = 0;
        while (ros::ok())
        {
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                ROS_ERROR_STREAM("Error: serial port closed unexpectedly");
                break;
            }
            xq_status.Refresh();//定时发布状态
            ii++;
            if(ii%5==0)
            {
              xq_diffdriver.sendcmd2();
            }
            r.sleep();
            //cout<<"run"<<endl;
        }

        quit:
        serial.close();

    } catch (std::exception& e) {
        ROS_ERROR_STREAM("Open " << port << " failed.");
        ROS_ERROR_STREAM("Exception: " << e.what());
    }

    ros::shutdown();
    return 0;
}
