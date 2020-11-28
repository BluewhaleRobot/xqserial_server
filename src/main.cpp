
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
    cout<<"welcome to xiaoqiang serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "xqserial_server");
    ros::start();

    //获取串口参数
    std::string port_car;
    ros::param::param<std::string>("~port_car", port_car, "/dev/stm32Car");
    int baud_car;
    ros::param::param<int>("~baud_car", baud_car, 115200);
    cout<<"port_car:"<<port_car<<" baud_car:"<<baud_car<<endl;

    std::string port_imu;
    ros::param::param<std::string>("~port_imu", port_imu, "/dev/stm32Imu");
    int baud_imu;
    ros::param::param<int>("~baud_imu", baud_imu, 115200);
    cout<<"port_imu:"<<port_imu<<" baud_car:"<<baud_imu<<endl;

    //获取小车机械参数
    double separation=0,radius=0;
    bool DebugFlag = false;
    ros::param::param<double>("~wheel_separation", separation, 0.42);
    ros::param::param<double>("~wheel_radius", radius, 0.086);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);

    double power_scale;
    ros::param::param<double>("~power_scale", power_scale, 1.0);

    double rot_dist,tran_dist;
    ros::param::param<double>("~rot_dist", rot_dist, -0.21);
    ros::param::param<double>("~tran_dist", tran_dist, -0.3);

    xqserial_server::StatusPublisher xq_status(separation,radius,power_scale);
    xq_status.setBarParams(rot_dist,tran_dist);
    //获取小车控制参数
    double max_speed,r_min;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 5.0);
    ros::param::param<double>("~r_min", r_min, 0.5);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    double angle_limit,x_limit,y_limit;
    ros::param::param<double>("~angle_limit", angle_limit, 1.6);
    ros::param::param<double>("~x_limit", x_limit, 1.2);
    ros::param::param<double>("~y_limit", y_limit, 0.2);
    // 初始化log发布者和语音发布者
    ros::NodeHandle mNH;
    ros::Publisher log_pub = mNH.advertise<xiaoqiang_log::LogRecord>("/xiaoqiang_log", 1, true);
    ros::Publisher audio_pub = mNH.advertise<std_msgs::String>("/xiaoqiang_tts/text", 1, true);

    try {
      CallbackAsyncSerial serial_car(port_car,baud_car);
      serial_car.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_car,&xq_status,_1,_2));

      CallbackAsyncSerial serial_imu(port_imu,baud_imu);
      serial_imu.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update_imu,&xq_status,_1,_2));

      xqserial_server::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial_car,&serial_imu,r_min);
      xq_diffdriver.setBarParams(angle_limit,tran_dist,x_limit,y_limit);
      boost::thread cmd2serialThread(& xqserial_server::DiffDriverController::run,&xq_diffdriver);

      const char driver_reset_cmd[8] = {(char)0xa6,(char)0x06,(char)0x00,(char)0x4f,(char)0x00,(char)0x81,(char)0x61,(char)0x6a}; //模式
      const char driver_canmode_cmd[8] = {(char)0xa6,(char)0x06,(char)0x00,(char)0x4f,(char)0x00,(char)0x01,(char)0x60,(char)0xca}; //模式
      const char driver1_speed_mode_cmd[8] = {(char)0x01,(char)0x06,(char)0x00,(char)0x51,(char)0x00,(char)0x03,(char)0x98,(char)0x1a}; //模式
      const char driver2_speed_mode_cmd[8] = {(char)0x02,(char)0x06,(char)0x00,(char)0x51,(char)0x00,(char)0x03,(char)0x98,(char)0x29}; //模式
      const char driver1_read_odom_cmd[8] = {(char)0x01,(char)0x03,(char)0x00,(char)0x83,(char)0x00,(char)0x02,(char)0x35,(char)0xe3}; //模式
      const char driver2_read_odom_cmd[8] = {(char)0x02,(char)0x03,(char)0x00,(char)0x83,(char)0x00,(char)0x02,(char)0x35,(char)0xd0}; //模式

      // send reset cmd
      char imu_resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};

      serial_imu.write(imu_resetCmd, 5);
      serial_car.write(driver_reset_cmd, 8);
      ros::Duration(1).sleep();

      //所有驱动器进入can操作模式
      serial_car.write(driver_canmode_cmd, 8);
      ros::Duration(1).sleep();
      ros::Rate r(50);//发布周期为50hz
      ros::WallTime last_movetime=ros::WallTime::now();
      static int i = 0;
      static int clear_try = 0;
      while (ros::ok())
      {
        if(serial_car.errorStatus() || serial_car.isOpen()==false)
        {
            cerr<<"Error: serial port closed unexpectedly"<<endl;
            break;
        }

        if(i%100 == 0)
        {
          //每隔2秒下发心跳包
          xq_diffdriver.sendHeartbag();
        }

        serial_car.write(driver1_read_odom_cmd,8);
        usleep(1000);//间隔1MS
        serial_car.write(driver2_read_odom_cmd,8);
        usleep(8000);//延时8MS，等待数据上传和处理
        xq_status.Refresh();//定时发布状态
        if(xq_status.car_status.driver_status == 0 && i%10==0)
        {
          //驱动器如果没有进入can模式则不断下发进入指令
          serial_car.write(driver_canmode_cmd, 8);
          usleep(1000);//间隔1MS
        }
        else
        {
          if(xq_status.car_status.driver_status == 1)
          {
            //如果没有进入速度模式，则下发速度模式指令
            if(i%10==0)
            {
              serial_car.write(driver1_speed_mode_cmd, 8);
              usleep(1000);//间隔1MS
              serial_car.write(driver2_speed_mode_cmd, 8);
              usleep(1000);//间隔1MS
            }
          }
          else
          {
            if(i%2==0)
            {
              xq_diffdriver.Refresh();//更新驱动速度 25hz
            }
          }
        }
        i++;
        r.sleep();
      }
      xq_diffdriver.send_release();
      usleep(10000);//延时10MS
      quit:
        serial_car.close();
        serial_imu.close();

    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Open " << port_car << " or "<< port_imu << " failed.");
      ROS_ERROR_STREAM("Exception: " << e.what());
      // 检查串口设备是否存在
      if (!exists(port_car))
      {
          // 发送语音提示消息
          std_msgs::String audio_msg;
          audio_msg.data = "未发现电机串口，请检查串口连接";
          audio_pub.publish(audio_msg);
          xiaoqiang_log::LogRecord log_record;
          log_record.collection_name = "exception";
          log_record.stamp = ros::Time::now();
          Json::Value record;
          record["type"] = "HARDWARE_ERROR";
          record["info"] = "电机串口设备未找到: " + port_car;
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
