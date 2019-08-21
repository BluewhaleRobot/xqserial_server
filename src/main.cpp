
#include "AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>
#include <json/json.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"
#include "xiaoqiang_log/LogRecord.h"

using namespace std;

inline bool exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

int main(int argc, char **argv)
{
    cout << "welcome to xiaoqiang serial server,please feel free at home!" << endl;

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
    double separation = 0, radius = 0;
    bool DebugFlag = false;
    ros::param::param<double>("~wheel_separation", separation, 0.68);
    ros::param::param<double>("~wheel_radius", radius, 0.0625);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    xqserial_server::StatusPublisher xq_status(separation, radius);

    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 2.0);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    // 初始化log发布者和语音发布者
    ros::NodeHandle mNH;
    ros::Publisher log_pub = mNH.advertise<xiaoqiang_log::LogRecord>("/xiaoqiang_log", 1, true);
    ros::Publisher audio_pub = mNH.advertise<std_msgs::String>("/xiaoqiang_tts/text", 1, true);

    try
    {
        CallbackAsyncSerial serial_left(port_left,baud_left);
        serial_left.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_left,&xq_status,_1,_2));

        CallbackAsyncSerial serial_right(port_right,baud_right);
        serial_right.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_right,&xq_status,_1,_2));

        xqserial_server::DiffDriverController xq_diffdriver(max_speed, cmd_topic, &xq_status,&serial_left,&serial_right);
        boost::thread cmd2serialThread(&xqserial_server::DiffDriverController::run, &xq_diffdriver);
        // send test flag
        char debugFlagCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'T'};
        if (DebugFlag)
        {
            ROS_INFO_STREAM("Debug mode Enabled");
            serial_left.write(debugFlagCmd, 5);
            serial_right.write(debugFlagCmd, 5);
        }
        // send reset cmd
        char resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};
        serial_left.write(resetCmd, 5);
        serial_right.write(resetCmd, 5);
        ros::Duration(0.5).sleep();

        ros::Rate r(100); //发布周期为50hz
        int i_num = 0;
        while (ros::ok())
        {
            if (serial_left.errorStatus() || serial_left.isOpen() == false)
            {
                ROS_ERROR_STREAM("Error: serial_left port closed unexpectedly");
                break;
            }
            if (serial_right.errorStatus() || serial_right.isOpen() == false)
            {
                ROS_ERROR_STREAM("Error: serial_right port closed unexpectedly");
                break;
            }
            xq_status.Refresh(); //定时发布状态
            i_num ++;
            if(i_num>10)
            {
              xq_diffdriver.checkStop();
              i_num = 0;
            }
            r.sleep();
            //cout<<"run"<<endl;
        }

    quit:
        serial_left.close();
        serial_right.close();
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM("Open " << port_left << " or "<< port_right << " failed.");
        ROS_ERROR_STREAM("Exception: " << e.what());
        // 检查串口设备是否存在
        if (!exists(port_left))
        {
            // 发送语音提示消息
            std_msgs::String audio_msg;
            audio_msg.data = "未发现左轮电机串口，请检查串口连接";
            audio_pub.publish(audio_msg);
            xiaoqiang_log::LogRecord log_record;
            log_record.collection_name = "exception";
            log_record.stamp = ros::Time::now();
            Json::Value record;
            record["type"] = "HARDWARE_ERROR";
            record["info"] = "左轮电机串口设备未找到: " + port_left;
            Json::FastWriter fastWriter;
            log_record.record = fastWriter.write(record);
            // 发送日志消息
            log_pub.publish(log_record);
        }
        if (!exists(port_right))
        {
            // 发送语音提示消息
            std_msgs::String audio_msg;
            audio_msg.data = "未发现右轮电机串口，请检查串口连接";
            audio_pub.publish(audio_msg);
            xiaoqiang_log::LogRecord log_record;
            log_record.collection_name = "exception";
            log_record.stamp = ros::Time::now();
            Json::Value record;
            record["type"] = "HARDWARE_ERROR";
            record["info"] = "右轮电机串口设备未找到: " + port_left;
            Json::FastWriter fastWriter;
            log_record.record = fastWriter.write(record);
            // 发送日志消息
            log_pub.publish(log_record);
        }
        ros::shutdown();
        return 1;
    }

    ros::shutdown();
    return 0;
}
