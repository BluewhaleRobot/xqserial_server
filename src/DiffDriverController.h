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
    geometry_msgs::Twist get_cmdTwist(void);
    void sendcmd2(const geometry_msgs::Twist &command);
    void check_faster_stop();
    void updateFastStopFlag(const std_msgs::Int32& fastStopmsg);
    void UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status);
    int speed_debug[2];
    ros::WallTime last_ordertime;
    bool DetectFlag_;
    bool fastStopFlag_;
 
private:
    double max_wheelspeed;//单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher* xq_status;
    CallbackAsyncSerial* cmd_serial;
    boost::mutex mMutex;
    bool MoveFlag;
    geometry_msgs::Twist  cmdTwist_;//小车自身坐标系
    boost::mutex mStausMutex_;
    galileo_serial_server::GalileoStatus galileoStatus_;
    float R_min_;
};

}
#endif // DIFFDRIVERCONTROLLER_H
