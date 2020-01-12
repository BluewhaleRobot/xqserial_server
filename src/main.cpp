
#include "AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"
#include <ros/console.h>
#include "xiaoqiang_log/LogRecord.h"
#include <json/json.h>
#include <std_msgs/String.h>

using namespace std;

inline bool exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}


int main(int argc, char **argv)
{
    cout<<"welcome to xiaoqiang serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "xqserial_server");
    ros::start();

    //获取串口参数
    std::string port_car_left;
    ros::param::param<std::string>("~port_car_left", port_car_left, "/dev/stm32Left");
    int baud_car_left;
    ros::param::param<int>("~baud_car_left", baud_car_left, 57600);
    cout<<"port_car_left:"<<port_car_left<<" baud_car_left:"<<baud_car_left<<endl;

    std::string port_car_right;
    ros::param::param<std::string>("~port_car_right", port_car_right, "/dev/stm32Right");
    int baud_car_right;
    ros::param::param<int>("~baud_car_right", baud_car_right, 57600);
    cout<<"port_car_right:"<<port_car_right<<" baud_car_right:"<<baud_car_right<<endl;

    std::string port_imu;
    ros::param::param<std::string>("~port_imu", port_imu, "/dev/stm32Imu");
    int baud_imu;
    ros::param::param<int>("~baud_imu", baud_imu, 115200);
    cout<<"port_imu:"<<port_imu<<" baud_car:"<<baud_imu<<endl;

    //获取小车机械参数
    double separation=0,radius=0;
    bool DebugFlag = false;

    ros::param::param<double>("~wheel_separation", separation, 0.38);
    ros::param::param<double>("~wheel_radius", radius, 0.065);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);

    double rot_dist,tran_dist;
    ros::param::param<double>("~rot_dist", rot_dist, -0.21);
    ros::param::param<double>("~tran_dist", tran_dist, -0.3);

    //获取小车控制参数
    double max_speed,r_min;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 5.0);
    ros::param::param<double>("~r_min", r_min, 0.5);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    radius = radius;
    max_speed = max_speed;

    double power_scale;
    ros::param::param<double>("~power_scale", power_scale, 1.0);
    xqserial_server::StatusPublisher xq_status(separation,radius,DebugFlag,power_scale);
    xq_status.setBarParams(rot_dist,tran_dist);

    // 初始化log发布者和语音发布者
    ros::NodeHandle mNH;
    ros::Publisher log_pub = mNH.advertise<xiaoqiang_log::LogRecord>("/xiaoqiang_log", 1, true);
    ros::Publisher audio_pub = mNH.advertise<std_msgs::String>("/xiaoqiang_tts/text", 1, true);

    try {
      CallbackAsyncSerial serial_car_left(port_car_left,baud_car_left);
      serial_car_left.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_car_left,&xq_status,_1,_2));

      CallbackAsyncSerial serial_car_right(port_car_right,baud_car_right);
      serial_car_right.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_car_right,&xq_status,_1,_2));

      CallbackAsyncSerial serial_imu(port_imu,baud_imu);
      serial_imu.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_imu,&xq_status,_1,_2));

      xqserial_server::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial_car_left,&serial_car_right,&serial_imu,r_min);
      boost::thread cmd2serialThread(& xqserial_server::DiffDriverController::run,&xq_diffdriver);


      // send test flag
      char debugFlagCmd[] = {(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01, 'T'};
      if(DebugFlag){
        std::cout << "Debug mode Enabled" << std::endl;
        serial_imu.write(debugFlagCmd, 5);
      }
      // send reset cmd
      char resetCmd[] = {(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01, 'I'};
      serial_imu.write(resetCmd, 5);
      ros::Duration(5).sleep();

      ros::Rate r(100);//发布周期为50hz
      int i=0;

      const char left_speed_mode_cmd1[4] = {(char)0x02,(char)0x00,(char)0xc4,(char)0xc6}; //模式
      const char left_speed_mode_cmd2[4] = {(char)0x0a,(char)0xff,(char)0xff,(char)0x08};//加减速度
      const char left_speed_mode_cmd3[4] = {(char)0x06,(char)0x00,(char)0x00,(char)0x06};//目标速度
      const char left_speed_mode_cmd4[4] = {(char)0x00,(char)0x00,(char)0x01,(char)0x01};//使能电机 锁轴
      const char left_speed_mode_cmd5[4] = {(char)0x00,(char)0x00,(char)0x00,(char)0x00};//松轴
      const char left_speed_mode_cmd6[4] = {(char)0x4a,(char)0x00,(char)0x00,(char)0x4a};//清除故障

      const char right_speed_mode_cmd1[4] = {(char)0x02,(char)0x00,(char)0xc4,(char)0xc6}; //模式
      const char right_speed_mode_cmd2[4] = {(char)0x0a,(char)0xff,(char)0xff,(char)0x08};//加减速度
      const char right_speed_mode_cmd3[4] = {(char)0x06,(char)0x00,(char)0x00,(char)0x06};//目标速度
      const char right_speed_mode_cmd4[4] = {(char)0x00,(char)0x00,(char)0x01,(char)0x01};//使能电机 锁轴
      const char right_speed_mode_cmd5[4] = {(char)0x00,(char)0x00,(char)0x00,(char)0x00};//松轴
      const char right_speed_mode_cmd6[4] = {(char)0x4a,(char)0x00,(char)0x00,(char)0x4a};//清除故障

      const char left_query1[4] = {(char)0x60,(char)0x00,(char)0x00,(char)0x60};//状态
      const char right_query1[4] = {(char)0x60,(char)0x00,(char)0x00,(char)0x60};//状态

      while (ros::ok())
      {
          if(serial_car_left.errorStatus() || serial_car_left.isOpen()==false)
          {
              cerr<<"Error: serial port car left closed unexpectedly"<<endl;
              break;
          }
          if(serial_car_right.errorStatus() || serial_car_right.isOpen()==false)
          {
              cerr<<"Error: serial port car right closed unexpectedly"<<endl;
              break;
          }
          if(serial_imu.errorStatus() || serial_imu.isOpen()==false)
          {
              cerr<<"Error: serial port imu closed unexpectedly"<<endl;
              break;
          }
          //先配置速度模式
          if(xq_status.car_status.left_driver_status==-1)
          {
            ROS_ERROR("enable left motor!");
            serial_car_left.write(left_speed_mode_cmd1,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_left.write(left_speed_mode_cmd2,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_left.write(left_speed_mode_cmd3,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_left.write(left_speed_mode_cmd4,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_left.write(left_query1,4);
            usleep(15000);//延时1MS，等待数据上传
            continue;
          }
          if(xq_status.car_status.left_driver_status>1 && xq_status.car_status.left_driver_status<0x80)
          {
            ROS_ERROR("clear left motor error!");
            serial_car_left.write(left_speed_mode_cmd5,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_left.write(left_speed_mode_cmd6,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_left.write(left_query1,8);
            usleep(15000);//延时1MS，等待数据上传
            continue;
          }

          if(xq_status.car_status.right_driver_status==-1)
          {
            ROS_ERROR("enable right motor!");
            serial_car_right.write(right_speed_mode_cmd1,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_right.write(right_speed_mode_cmd2,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_right.write(right_speed_mode_cmd3,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_right.write(right_speed_mode_cmd4,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_right.write(right_query1,4);
            usleep(15000);//延时1MS，等待数据上传
            continue;
          }
          if(xq_status.car_status.right_driver_status>1 && xq_status.car_status.right_driver_status<0x80)
          {
            ROS_ERROR("clear right motor error!");
            serial_car_right.write(right_speed_mode_cmd5,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_right.write(right_speed_mode_cmd6,4);
            usleep(15000);//延时1MS，等待数据上传
            serial_car_right.write(right_query1,4);
            usleep(15000);//延时1MS，等待数据上传
            continue;
          }
          i++;
          xq_status.Refresh();//定时发布状态
          if(i%100==0)
          {
            serial_car_left.write(left_query1,4);
            serial_car_right.write(right_query1,4);
          }
          r.sleep();
      }
      quit:
      serial_car_left.close();
      serial_car_right.close();
      serial_imu.close();

    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Open " << port_car_left << " or "<< port_car_right << " or "<< port_imu << " failed.");
      ROS_ERROR_STREAM("Exception: " << e.what());
      // 检查串口设备是否存在
      if (!exists(port_car_left))
      {
          // 发送语音提示消息
          std_msgs::String audio_msg;
          audio_msg.data = "未发现左侧电机串口，请检查串口连接";
          audio_pub.publish(audio_msg);
          xiaoqiang_log::LogRecord log_record;
          log_record.collection_name = "exception";
          log_record.stamp = ros::Time::now();
          Json::Value record;
          record["type"] = "HARDWARE_ERROR";
          record["info"] = "左侧电机串口设备未找到: " + port_car_left;
          Json::FastWriter fastWriter;
          log_record.record = fastWriter.write(record);
          // 发送日志消息
          log_pub.publish(log_record);
      }
      if (!exists(port_car_right))
      {
          // 发送语音提示消息
          std_msgs::String audio_msg;
          audio_msg.data = "未发现右侧电机串口，请检查串口连接";
          audio_pub.publish(audio_msg);
          xiaoqiang_log::LogRecord log_record;
          log_record.collection_name = "exception";
          log_record.stamp = ros::Time::now();
          Json::Value record;
          record["type"] = "HARDWARE_ERROR";
          record["info"] = "右侧电机串口设备未找到: " + port_car_right;
          Json::FastWriter fastWriter;
          log_record.record = fastWriter.write(record);
          // 发送日志消息
          log_pub.publish(log_record);
      }
      if (!exists(port_imu))
      {
          // 发送语音提示消息
          std_msgs::String audio_msg;
          audio_msg.data = "未发现传感器串口，请检查串口连接";
          audio_pub.publish(audio_msg);
          xiaoqiang_log::LogRecord log_record;
          log_record.collection_name = "exception";
          log_record.stamp = ros::Time::now();
          Json::Value record;
          record["type"] = "HARDWARE_ERROR";
          record["info"] = "传感器串口设备未找到: " + port_imu;
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
