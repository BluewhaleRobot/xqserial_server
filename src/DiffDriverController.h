#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
#include <ros/ros.h>
#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <std_msgs/Bool.h>

namespace xqserial_server
{

class DiffDriverController
{
public:
    DiffDriverController();
    DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_car_,CallbackAsyncSerial* cmd_serial_imu_);

    void run();
    void dealCmd_vel(const geometry_msgs::Twist& command);
    void imuCalibration(const std_msgs::Bool& calFlag);
    void setStatusPtr(StatusPublisher& status);
    void updateMoveFlag(const std_msgs::Bool& moveFlag);
    void updateBarDetectFlag(const std_msgs::Bool& DetectFlag);
    bool get_speed(int16_t & left_speed ,int16_t & right_speed);
    void send_speed();
    void keep_speed();
private:
    double max_wheelspeed;//单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher* xq_status;
    CallbackAsyncSerial* cmd_serial_car;
    CallbackAsyncSerial* cmd_serial_imu;
    boost::mutex mMutex;
    bool MoveFlag;
    bool BarFlag;
    int16_t left_speed_;
    int16_t right_speed_;
    bool send_flag_;
};

}
#endif // DIFFDRIVERCONTROLLER_H
