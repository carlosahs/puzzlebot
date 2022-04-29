#include <ros/ros.h>
#include "puzzlebot_rviz/PuzzleBotSim.hpp"

int main(int arc, char ** argv)
{
    ROS_INFO("[PuzzleBot] Starting robot top level node");
    ros::init(arc,argv,"puzzlebot_sim");

    ros::NodeHandle nh;
    
    std::string id = "";
    if (arc > 1)
        id = argv[1];
    else{
        ROS_WARN("No robot id given, creating robot with id 0");
        id = "0";
    }
    PuzzleBotSimulator::PuzzleBotSim puzzle_bot_sim(nh,id);

    ros::Rate loop_rate(puzzle_bot_sim.GetLoopRate());

    ROS_INFO("[PuzzleBot] Robot top level node started for robot id %s at loop rate %d",id.c_str(),puzzle_bot_sim.GetLoopRate());

    while(ros::ok()){
        ros::spinOnce();

        puzzle_bot_sim.Run();

        loop_rate.sleep();
    }
    
}