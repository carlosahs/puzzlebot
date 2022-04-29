#ifndef PUZZLEBOT_HPP
#define PUZZLEBOT_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

namespace PuzzleBotSimulator{

class PuzzleBot{
  public:

    PuzzleBot(ros::NodeHandle& nodeHandle,std::string prefix);
    int GetLoopRate();
    void Run();

  private:

    void LeftTorqCallback(const std_msgs::Float32::ConstPtr& msg);
    void RightTorqCallback(const std_msgs::Float32::ConstPtr& msg);

    ros::NodeHandle* nodeHandle;
    ros::Subscriber sub_left_torq,sub_right_torq;
    ros::Publisher pub_cmd_vel, pub_etaL,pub_etaR;
    int loop_rate;

    std::map<std::string,std::string> topics;
    std::map<std::string,double> params;

    double Vx,Wz,etaL,etaR,frR,frL,tauR,tauL,v11,v12,v21,v22,mInv11,mInv12,mInv21,mInv22,dt;
    ros::Time current_time, last_time;
};

}

#endif