#include <ros/ros.h>
#include "puzzlebot_rviz/PuzzleBot.hpp"

int main(int arc, char ** argv){
    ROS_INFO("[PuzzleBot] Starting robot simulation node");
    ros::init(arc,argv,"puzzlebot");

    ros::NodeHandle nh;

    std::string prefix = "";
    if (arc > 1)
        prefix = argv[1];
    else{
        prefix = "/puzzlebot_sim/puzzlebot0";
        ROS_WARN("No prefix given, creating puzzlebot with prefix %s",prefix.c_str());
    }
    PuzzleBotSimulator::PuzzleBot puzzle_bot(nh,prefix);

    ros::Rate loop_rate(puzzle_bot.GetLoopRate());

    ROS_INFO("[PuzzleBot] Robot simulation node started sharing on %s at loop rate %d",prefix.c_str(),puzzle_bot.GetLoopRate());

    while(ros::ok()){
        ros::spinOnce();

        puzzle_bot.Run();

        loop_rate.sleep();
    }
}