#include "puzzlebot_rviz/PuzzleBot.hpp"
#include "puzzlebot_rviz/Shared.hpp"
namespace PuzzleBotSimulator{

PuzzleBot::PuzzleBot(ros::NodeHandle& nodeHandle,std::string prefix){
    // Check for puzzlebot level loop rate, otherwise use simulator level loop rate
    if(!nodeHandle.getParam("/puzzlebot_sim/puzzlebot/loop_rate",loop_rate))
        nodeHandle.param("/puzzlebot_sim/loop_rate",loop_rate,(int)1000);

    // Import topics from ROS parameter server
    topics = FetchTopics("/puzzlebot_sim/puzzlebot",nodeHandle);

    //Check if there are active motors, if so set torque and eta topics to the motor topics
    if(nodeHandle.getParam("/puzzlebot_sim/puzzlebot/motor/topics/torque",topics["torque"])){
        topics["right_torque"] = "motor2/" + topics["torque"];
        topics["left_torque"] = "motor1/" + topics["torque"];
    }
    if(nodeHandle.getParam("/puzzlebot_sim/puzzlebot/motor/topics/eta",topics["eta"])){
        topics["right_eta"] = "motor1/" + topics["eta"];
        topics["left_eta"] = "motor2/" + topics["eta"];
    }
    if(!CheckTopics(topics,{"left_torque","right_torque","left_eta","right_eta","cmd_vel"}))
        ros::shutdown();
    topics = InsertPrefix(topics,prefix);
    // Import parameters from ROS parameter server
    params = FetchParameters("/puzzlebot_sim/puzzlebot",nodeHandle);
    std::vector<std::string> required_params = {"M","mw","I0","d","L","Im","Rr","Iw","Rl","Cs_r","Cs_l","Cv_l","Cv_r"};
    if(!CheckParameters(params,required_params))
        ros::shutdown();

    //Local variables for comupation of inv(M)
    double Mt,It,m11,m12,m21,m22,detM;
    
    //Initialise inputs, outputs and other inital-zero variables
    tauL = 0;   //  Wheel torques
    tauR = 0;
    Vx = 0;     //  Robot linear and angular velocities
    Wz = 0;
    etaL = 0;   //  Wheel angular velocities 
    etaR = 0;
    frR = 0;    //  Wheel Friction
    frL = 0;

    //Create the Inertia Matrix M for the Lagrance model and store its inverse
    Mt = params["M"] + 2 * params["mw"];
    It = params["I0"] + params["M"] * pow(params["d"],2) + 2 * params["mw"] * pow(params["L"],2) + 2 * params["Im"];

    m11 = (pow(params["Rr"],2) / 4) * (Mt + It / pow(params["L"],2)) + params["Iw"];
    m12 = params["Rr"] * params["Rl"] / 4 * (Mt - It / pow(params["L"],2));
    m21 = m12;
    m22 = (pow(params["Rl"],2) / 4) * (Mt + It / pow(params["L"],2)) + params["Iw"]; 

    detM = m11 * m22 - m12 * m21;

    mInv11 = m22 / detM;
    mInv12 = -m12 / detM;
    mInv21 = -m21 / detM;
    mInv22 = m11 / detM;

    //Create the coriolis matrix for the lagrange model 
    v11 = 0;
    v12 = params["M"] * params["d"] * params["Rr"] * params["Rl"] / (2 * params["L"]);
    v21 = -v12;
    v22 = 0;

    // Setup ROS subsribers
    sub_left_torq = nodeHandle.subscribe(topics["left_torque"],10,&PuzzleBot::LeftTorqCallback,this);
    sub_right_torq = nodeHandle.subscribe(topics["right_torque"],10,&PuzzleBot::RightTorqCallback,this);

    // Setup ROS publishers
    pub_cmd_vel = nodeHandle.advertise<geometry_msgs::Twist>(topics["cmd_vel"],1000);
    pub_etaR = nodeHandle.advertise<std_msgs::Float32>(topics["left_eta"],1000);
    pub_etaL = nodeHandle.advertise<std_msgs::Float32>(topics["right_eta"],1000);

    srand (time(NULL));
}

void PuzzleBot::LeftTorqCallback(const std_msgs::Float32::ConstPtr& msg){
    if(fabs(msg->data) < 0.07 && fabs(etaL) < 0.2)
	    tauL = 0.0;
    else
   	    tauL = msg->data;
}

void PuzzleBot::RightTorqCallback(const std_msgs::Float32::ConstPtr& msg){
    if(fabs(msg->data) < 0.07 && fabs(etaR) < 0.2)
	    tauR = 0.0;
    else
	    tauR = msg->data;
}

int PuzzleBot::GetLoopRate(){
    return loop_rate;
}

void PuzzleBot::Run(){
    //  Update Time period
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;

    double etaR_dot,etaL_dot;

    //  Compute friction for each wheel
    frR = params["Cs_r"] * sign(etaR) + params["Cv_r"] * etaR;
    frL = params["Cs_l"] * sign(etaL) + params["Cv_l"] * etaL;

    //  Use the Lagrangian formula for each wheel to comput acceleration
    etaR_dot = mInv11 * (tauR - Wz * v12 * etaL - frR) + mInv12 * (tauL - Wz * v21 * etaR - frL);
    etaL_dot = mInv21 * (tauL - Wz * v12 * etaL - frL) + mInv22 * (tauL - Wz * v21 * etaR - frL);

    // Integrate to get velocity
    etaR += etaR_dot * dt;
    etaL += etaL_dot * dt;

    // Publish Velocity
    std_msgs::Float32 etaR_msg,etaL_msg;
    etaR_msg.data = etaR;
    etaL_msg.data = etaL;

    pub_etaR.publish(etaR_msg);
    pub_etaL.publish(etaL_msg);

    //Compute robot velocities
    double etaR_n = etaR + 0.01*(2.0*(double)rand()/RAND_MAX - 1.0);
    double etaL_n = etaL + 0.01*(2.0*(double)rand()/RAND_MAX - 1.0);
    Vx = (params["Rr"] * etaR_n + params["Rl"] * etaL_n) / 2.0;
    Wz = (params["Rr"] * etaR_n - params["Rl"] * etaL_n) / params["L"];

    //Publish Robot Velocities
    geometry_msgs::Twist msg;

    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;

    msg.linear.x = Vx;
    msg.angular.z = Wz;

    pub_cmd_vel.publish(msg);
}

}
