#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
#include <ros/ros.h>
#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_listener.h>

namespace xqserial_server
{

class DiffDriverController
{
public:
    DiffDriverController();
    DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_);
    void run();
    void sendcmd(const geometry_msgs::Twist& command);
    void imuCalibration(const std_msgs::Bool& calFlag);
    void setStatusPtr(StatusPublisher& status);
    void updateMoveFlag(const std_msgs::Bool& moveFlag);
    void updateBarDetectFlag(const std_msgs::Bool& DetectFlag);
    void updateSonar1(const sensor_msgs::Range& range_msg);
    void updateSonar2(const sensor_msgs::Range& range_msg);
    void updateSonar3(const sensor_msgs::Range& range_msg);
    void updateSonar4(const sensor_msgs::Range& range_msg);
    void getSonarData(float (&ranges)[4],float (&view_angles)[4]);
    bool getSonarTf(float (&tf_angles)[4],float (&tf_xs)[4],float (&tf_ys)[4]);
    void setSonarTf(tf::StampedTransform &transform1,tf::StampedTransform &transform2,tf::StampedTransform &transform3,tf::StampedTransform &transform4);
    void sendcmd2();
private:
    double max_wheelspeed;//单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher* xq_status;
    CallbackAsyncSerial* cmd_serial;
    boost::mutex mMutex;
    boost::mutex mMutex_range;
    float ranges_[4];
    float view_angles_[4];

    bool MoveFlag;
    bool detectFlag_;
    tf::StampedTransform transform1_;
    tf::StampedTransform transform2_;
    tf::StampedTransform transform3_;
    tf::StampedTransform transform4_;
    bool sonarTf_ready_;
    float speed_last_[2];
};

}
#endif // DIFFDRIVERCONTROLLER_H
