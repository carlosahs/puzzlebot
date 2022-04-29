#include <ros/ros.h>
#include "puzzlebot_rviz/Motor.hpp"
#include <std_msgs/String.h>
int main(int arc, char ** argv)
{
    ROS_INFO("[PuzzleBot] Starting motor simulation node");
    ros::init(arc,argv,"puzzlebot_sim");

    ros::NodeHandle nh;
    std::string prefix = "";
    if (arc > 1)
        prefix = argv[1];
    else{
        prefix = "/puzzlebot_sim/puzzlebot0/motor0/";
        ROS_WARN("No prefix given, creating motor with prefix %s",prefix.c_str());
    }
    PuzzleBotSimulator::Motor motor(nh,prefix);
    
    ros::Rate loop_rate(motor.GetLoopRate());

    ROS_INFO("[PuzzleBot] Motor simulation node started sharing on %s at loop rate %d",prefix.c_str(),motor.GetLoopRate());

    while(ros::ok()){
        ros::spinOnce();

        motor.Run();

        loop_rate.sleep();
    }

}