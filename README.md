# xqserial_server
xiaoqiang motor driver and low level ROS api package
    
##input topic
      name                                  type        
    /cmd_vel                         geometry_msgs/Twist
      
##output topic
       name                                 type                 frate
    /xqserial_server/Odom            nav_msgs/Odometry            50hz
    /xqserial_server/Pose2D          geometry_msgs/Pose2D         50hz
    /xqserial_server/Power           std_msgs/Float64             50hz
    /xqserial_server/StatusFlag      std_msgs/Int32               50hz
    /xqserial_server/Twist           geometry_msgs/Twist          50hz
    /tf                              odom-->base_footprint        50hz
    /tf_static                       base_footprint-->base_link   100hz
    
##input param   
       name                            default
    port                             /dev/ttyUSB0
    baud                               115200
    wheel_separation                    0.37
    wheel_radius                        0.06
    debug_flag                          false
    max_speed                           2.0
    cmd_topic                           cmd_vel

##Usage:
###download to xiaoqiang ros workspace
```
cd ~/Documents/ros/src
git clone https://github.com/BlueWhaleRobot/xqserial_server.git 
cd ..
catkin_make
```
###Quickstart
```
roslaunch xqserial_server xqserial.launch
```
##Made with :heart: by BlueWhale Tech corp.
    
    
小强电机驱动和相关的ROS底层驱动程序。  
##使用方法：
####安装到小强ROS工作目录
```
cd ~/Documents/ros/src
git clone https://github.com/BlueWhaleRobot/xqserial_server.git 
cd ..
catkin_make
```
###直接启动
```
roslaunch xqserial_server xqserial.launch
```
    
##由蓝鲸科技精 :heart: 制作。
