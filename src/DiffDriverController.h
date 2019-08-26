#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
#include <ros/ros.h>
#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <std_msgs/Bool.h>
#include "galileo_serial_server/GalileoStatus.h"

namespace xqserial_server
{

class DiffDriverController
{
public:
    DiffDriverController();
    DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_car_,CallbackAsyncSerial* cmd_serial_imu_,double r_min);

    void run();
    void dealCmd_vel(const geometry_msgs::Twist& command);
    void imuCalibration(const std_msgs::Bool& calFlag);
    void updateMoveFlag(const std_msgs::Bool& moveFlag);
    void updateBarDetectFlag(const std_msgs::Bool& DetectFlag);
    void updateFastStopFlag(const std_msgs::Int32& fastStopmsg);
    void send_speed();
    void filterSpeed();
    void send_stop();
    void send_fasterstop();
    void send_release();
    ros::WallTime last_ordertime;
    void UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status);

private:
    double max_wheelspeed;//单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher* xq_status;
    CallbackAsyncSerial* cmd_serial_car;
    CallbackAsyncSerial* cmd_serial_imu;
    boost::mutex mMutex;
    bool MoveFlag;
    bool BarFlag;
    bool fastStopFlag_;
    bool updateOrderflag_;
    int16_t left_speed_;
    int16_t right_speed_;
    float linear_x_;
    float theta_z_;
    bool send_flag_;
    boost::mutex mStausMutex_;
    galileo_serial_server::GalileoStatus galileoStatus_;
    float R_min_;
};

}
#endif // DIFFDRIVERCONTROLLER_H
