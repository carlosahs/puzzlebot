#ifndef PUZZLEBOTSIM_HPP
#define PUZZLEBOTSIM_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include "puzzlebot_rviz/PIDController.hpp"

namespace PuzzleBotSimulator{

class PuzzleBotSim{
  public:

    PuzzleBotSim(ros::NodeHandle& nodeHandle,std::string id);
    int GetLoopRate();
    void Run();

  private:

    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void SetCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void SetWrCallback(const std_msgs::Float32::ConstPtr& msg);
    void SetWlCallback(const std_msgs::Float32::ConstPtr& msg);
    void SetPWMrCallback(const std_msgs::Float32::ConstPtr& msg);
    void SetPWMlCallback(const std_msgs::Float32::ConstPtr& msg);
    void WrCallback(const std_msgs::Float32::ConstPtr& msg);
    void WlCallback(const std_msgs::Float32::ConstPtr& msg);
    ros::NodeHandle* nodeHandle;
    ros::Subscriber sub_cmd_vel,set_cmd_vel,set_wL,set_wR,set_pwmL,set_pwmR,sub_wR,sub_wL;
    ros::Publisher pub_odom,pub_pwmR,pub_pwmL,pub_w,pub_v;
    tf::TransformBroadcaster odom_broadcaster;
    int loop_rate;

    std::map<std::string,std::string> topics;
    std::map<std::string,double> params;
    double x,y,theta,x_n,y_n,theta_n,Vx,Vy,W,Vx_in,Vy_in,W_in,cmd_wR,cmd_wL,cmd_pwmR,cmd_pwmL,sys_wR,sys_wL;
    ros::Time current_time,last_time;

    PIDController PWML_controller,PWMR_controller;
};

}

#endif