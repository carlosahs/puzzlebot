#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace PuzzleBotSimulator{

class Motor{
  public:

    Motor(ros::NodeHandle& nodeHandle,std::string prefix);
    int GetLoopRate();
    void Run();

  private:
  
    void PwmSetCallback(const std_msgs::Float32::ConstPtr& msg);
    void EtaSetCallback(const std_msgs::Float32::ConstPtr& msg);

    ros::NodeHandle* nodeHandle;
    ros::Subscriber sub_pwm,sub_eta;
    ros::Publisher pub_torq;
    int loop_rate;

    std::map<std::string,std::string>topics;
    std::map<std::string,double>params;

    std::string prefix;
    double dt,Va,eta,tau,I;
    ros::Time current_time,last_time;
    
};

}
#endif