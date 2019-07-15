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
    DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_,double r_min);
    void run();
    void sendcmd(const geometry_msgs::Twist& command);
    void imuCalibration(const std_msgs::Bool& calFlag);
    void setStatusPtr(StatusPublisher& status);
    void updateMoveFlag(const std_msgs::Bool& moveFlag);
    void updateBarDetectFlag(const std_msgs::Bool& DetectFlag);
    void updateElevator(const std_msgs::Int32& elevatormsg);
    bool checkStop();
    void send_speed();
    void filterSpeed();
    geometry_msgs::Twist get_cmdTwist(void);
    void updateFastStopFlag(const std_msgs::Int32& fastStopmsg);
    int speed_debug[2];
    ros::WallTime last_ordertime;
    bool DetectFlag_;
    bool stopFlag_;
    void UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status);
private:
    double max_wheelspeed;//单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher* xq_status;
    CallbackAsyncSerial* cmd_serial;
    boost::mutex mMutex;
    bool MoveFlag;
    geometry_msgs::Twist  cmdTwist_;//小车自身坐标系
    float linear_x_;
    float theta_z_;
    boost::mutex mStausMutex_;
    galileo_serial_server::GalileoStatus galileoStatus_;
    float R_min_;
};

}
#endif // DIFFDRIVERCONTROLLER_H
