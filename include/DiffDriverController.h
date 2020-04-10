#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
#include <ros/ros.h>
#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <std_msgs/Bool.h>
#include "galileo_serial_server/GalileoStatus.h"
#include <opencv2/core/core.hpp>

namespace xqserial_server
{

class DiffDriverController
{
  public:
    DiffDriverController();
    DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *xq_status_, CallbackAsyncSerial *cmd_serial_,double r_min);
    void run();
    void sendcmd(const geometry_msgs::Twist &command);
    void imuCalibration(const std_msgs::Bool &calFlag);
    void setStatusPtr(StatusPublisher &status);
    void updateMoveFlag(const std_msgs::Bool &moveFlag);
    void updateBarDetectFlag(const std_msgs::Bool &DetectFlag);
    void UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status);
    void dealCalibrateS();
    void linearSolve(const cv::Mat & A, const cv::Mat & b, cv::Mat & x);
    void updateCalibFlag(const std_msgs::Bool &calibFlag);
  private:
    double max_wheelspeed; //单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher *xq_status;
    CallbackAsyncSerial *cmd_serial;
    boost::mutex mMutex;
    bool MoveFlag;
    boost::mutex mStausMutex_;
    galileo_serial_server::GalileoStatus galileoStatus_;
    float R_min_;

    boost::mutex mMutex_calibrate;
    bool mcalibrate_flag;
    cv::Mat mA;
    cv::Mat mb;
    cv::Mat mx;
    int mcurrent_step;
    int mstep_now; // 0 run, 1 stop and wait, 2 end
    bool mt1_flag;
    bool mt2_flag;

};

} // namespace xqserial_server
#endif // DIFFDRIVERCONTROLLER_H
