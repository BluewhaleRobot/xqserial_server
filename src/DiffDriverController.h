#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
#include <ros/ros.h>
#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <std_msgs/Bool.h>
#include "galileo_serial_server/GalileoStatus.h"
#include "galileo_serial_server/GalileoNativeCmds.h"
#include "sensor_msgs/LaserScan.h"
#include <Eigen/Core>

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
    void updateFastStopFlag(const std_msgs::Int32& fastStopmsg);
    void UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status);
    bool dealBackSwitch();
    void send_speed();
    void UpdateSpeed();
    void filterGoal();
    void updateC2C4();
    void Refresh();
    void updateScan(const sensor_msgs::LaserScan& scan_in);
    void setBarParams(double angle_limit,double tran_dist, double x_limit, double y_limit)
    {
      boost::mutex::scoped_lock lock(mScanMutex_);
      angle_limit_ = angle_limit;
      tran_dist_ = tran_dist;
      x_limit_ = x_limit;
      y_limit_ = y_limit;
      scan_min_dist_ = x_limit_*2;
      move_forward_flag_ = true;
    }
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
    bool back_touch_falg_;
    ros::NodeHandle mNH_;
    ros::Publisher mgalileoCmdsPub_;
    ros::WallTime last_touchtime_;
    
    float linear_x_current_;
    float theta_z_current_;

    float linear_x_last_;
    float theta_z_last_;

    float linear_x_goal_;
    float theta_z_goal_;
    float R_goal_;

    float acc_vx_max_;
    float acc_wz_max_;

    float acc_vx_;
    float acc_wz_;
    float acc_vx_set_;
    float acc_wz_set_;

    //激光雷达避障
    boost::mutex mScanMutex_;
    float angle_limit_; //角度检查范围
    float tran_dist_; //安全距离
    float x_limit_; //最远距离
    float y_limit_; //车宽一半长度
    float move_forward_flag_; //允许前进标志
    float scan_min_dist_;//当前最近的雷达障碍物距离，小于连续3个点会被过滤

    float angle_min_;
    float angle_max_;
    Eigen::ArrayXXd co_sine_map_;
    std::vector<double> R_laserscan_;  //laserscan坐标系到base_footprint坐标系的转换
    std::vector<double> T_laserscan_;
    ros::WallTime last_scantime_;
};

}
#endif // DIFFDRIVERCONTROLLER_H
