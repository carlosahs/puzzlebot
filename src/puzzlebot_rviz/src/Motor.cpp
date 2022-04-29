#include "puzzlebot_rviz/Motor.hpp"
#include "puzzlebot_rviz/Shared.hpp"
namespace PuzzleBotSimulator{

Motor::Motor(ros::NodeHandle& nodeHandle,std::string prefix){

    // Check for motor loop rate, then puzzlebot loop rate, then simulator loop rate
    if(!nodeHandle.getParam("/puzzlebot_sim/puzzlebot/motor/loop_rate",loop_rate)){
        if(!nodeHandle.getParam("/puzzlebot_sim/puzzlebot/loop_rate",loop_rate)){
            nodeHandle.param("/puzzlebot_sim/loop_rate",loop_rate,(int)1000);
        }
    }
    // Import topics from ROS parameter server
    topics = FetchTopics("/puzzlebot_sim/puzzlebot/motor",nodeHandle);
    if (!CheckTopics(topics,{"pwm","eta","torque"}))
        ros::shutdown();
    topics = InsertPrefix(topics,prefix);
    
    // Import parameters from ROS parameters server
    params = FetchParameters("/puzzlebot_sim/puzzlebot/motor",nodeHandle);
    if(!CheckParameters(params,{"KT","Ke","Ra","La","tr","miu","tau_fr","eta","I","Vs"}))
        ros::shutdown();

    // Setup Subscribers
    sub_pwm = nodeHandle.subscribe(topics["pwm"],10,&Motor::PwmSetCallback,this);
    sub_eta = nodeHandle.subscribe(topics["eta"],10,&Motor::EtaSetCallback,this);

    // Setup Publishers
    pub_torq = nodeHandle.advertise<std_msgs::Float32>(topics["torque"],1000);

    //Initialise Variables
    Va = 0;
    tau = 0;
    I = params["I"];
    eta = params["eta"];

    current_time = ros::Time::now();
    last_time = ros::Time::now();
}

int Motor::GetLoopRate(){
    return loop_rate;
}

void Motor::PwmSetCallback(const std_msgs::Float32::ConstPtr& msg){
    Va = sign(msg->data) * sqrt(fabs(msg->data)) * params["Vs"];
}

void Motor::EtaSetCallback(const std_msgs::Float32::ConstPtr& msg){
    eta = msg->data;
}

void Motor::Run(){
    //Reference: https://doi.org/10.4172/2168-9695.1000107

    //Compute time period
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;

    //Compute Torque
    double I_dot = (Va - params["Ke"] * params["tr"] * eta - params["Ra"] * I) / params["La"];     
    I += dt*I_dot;
    tau = params["tr"] * params["miu"] * (params["KT"] * I - params["tau_fr"]);

    //Publish torque
    std_msgs::Float32 msg;
    msg.data = tau;
    pub_torq.publish(msg);
}

}
