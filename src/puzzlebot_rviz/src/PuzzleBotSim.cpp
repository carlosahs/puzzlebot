#include "puzzlebot_rviz/PuzzleBotSim.hpp"
#include "puzzlebot_rviz/Shared.hpp"
namespace PuzzleBotSimulator{

PuzzleBotSim::PuzzleBotSim(ros::NodeHandle& nodeHandle, std::string id){
    params = FetchParameters("/puzzlebot_sim",nodeHandle);
    std::map<std::string,double> puzzlebot_params = FetchParameters("/puzzlebot_sim/puzzlebot",nodeHandle);
    params.insert(puzzlebot_params.begin(),puzzlebot_params.end());
    topics = FetchTopics("/puzzlebot_sim",nodeHandle);
    if(!CheckParameters(params,{"mode"}))
        ros::shutdown();
    if(params["mode"] < 3 && !CheckParameters(params,{"Kp","Ti","Td","L","Rr","Rl"}))
        ros::shutdown();

    std::vector<std::string> required_topics = {"cmd_vel_input","odom"};
    if(params["mode"] == 1)
        required_topics.push_back({"cmd_vel"});
    if(params["mode"] == 2)
        required_topics.push_back("cmd_wL");
        required_topics.push_back("cmd_wR");
    if(params["mode"] == 3){
        required_topics.push_back("cmd_pwmL");
        required_topics.push_back("cmd_pwmR");
    }
    if(!CheckTopics(topics,required_topics))
        ros::shutdown();
    
    std::string prefix = "/puzzlebot_sim" + id + "/";
    topics = InsertPrefix(topics,prefix);
    
    if(nodeHandle.getParam("/puzzlebot_sim/puzzlebot/topics/cmd_vel",topics["cmd_vel_input"]))
        topics["cmd_vel_input"] = prefix + "puzzlebot/" + topics["cmd_vel_input"];
    else
        ROS_WARN("No puzzlebot nodes detected, simulation may not work correctly");
    
    if(nodeHandle.getParam("/puzzlebot_sim/puzzlebot/motor/topics/pwm",topics["pub_pwm"])){
        topics["pub_pwmR"] = prefix + "puzzlebot/motor1/" + topics["pub_pwm"];
        topics["pub_pwmL"] = prefix + "puzzlebot/motor2/" + topics["pub_pwm"];
        pub_pwmR = nodeHandle.advertise<std_msgs::Float32>(topics["pub_pwmR"],10);
        pub_pwmL = nodeHandle.advertise<std_msgs::Float32>(topics["pub_pwmL"],10);
    }else{
        ROS_WARN("No motor nodes detected, simulation may not work correctly");
    }

    if(nodeHandle.getParam("/puzzlebot_sim/puzzlebot/motor/topics/eta",topics["eta"])){
        topics["etaR"] = prefix + "puzzlebot/motor1/" + topics["eta"];
        topics["etaL"] = prefix + "puzzlebot/motor2/" + topics["eta"];
        sub_wR = nodeHandle.subscribe<std_msgs::Float32>(topics["etaR"],10,&PuzzleBotSim::WrCallback,this);
        sub_wL = nodeHandle.subscribe<std_msgs::Float32>(topics["etaL"],10,&PuzzleBotSim::WlCallback,this);
    }
    
    nodeHandle.param("/puzzlebot_sim/parameters/loop_rate",loop_rate,(int)100);

    sub_cmd_vel = nodeHandle.subscribe(topics["cmd_vel_input"],10,&PuzzleBotSim::CmdVelCallback,this);

    pub_odom = nodeHandle.advertise<nav_msgs::Odometry>(topics["odom"],1000);
    
    if(params["mode"] == 1){
        set_cmd_vel = nodeHandle.subscribe(topics["cmd_vel"],10,&PuzzleBotSim::SetCmdVelCallback,this);
    }
    else if(params["mode"] == 2){
        PWML_controller.SetParameters(params["Kp"],params["Ti"],params["Td"]);
        PWML_controller.SetControlLimits(-1,1);
        PWMR_controller.SetParameters(params["Kp"],params["Ti"],params["Td"]);
        PWMR_controller.SetControlLimits(-1,1);
        set_wL = nodeHandle.subscribe(topics["cmd_wL"],10,&PuzzleBotSim::SetWlCallback,this);
        set_wR = nodeHandle.subscribe(topics["cmd_wR"],10,&PuzzleBotSim::SetWrCallback,this);
    }
    else if(params["mode"] == 3){
        set_pwmR = nodeHandle.subscribe(topics["cmd_pwmR"],10,&PuzzleBotSim::SetPWMrCallback,this);
        set_pwmL = nodeHandle.subscribe(topics["cmd_pwmL"],10,&PuzzleBotSim::SetPWMlCallback,this);
    }
    else{
        ROS_ERROR("Invalid mode, valid modes are 1, 2, 3");
        ros::shutdown();
    }
    x = 0;
    y = 0;
    theta = 0;

    Vx = 0;
    Vy = 0;
    W = 0;

    Vx_in = 0;
    Vy_in = 0;
    W_in = 0;

    cmd_pwmR = 0;
    cmd_pwmL = 0;

    cmd_wL = 0;
    cmd_wR = 0;

    sys_wL = 0;
    sys_wR = 0;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //srand (time(NULL));

    pub_v = nodeHandle.advertise<std_msgs::Float32>("/v",100);
    pub_w = nodeHandle.advertise<std_msgs::Float32>("/w",100);
}

void PuzzleBotSim::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    Vx_in = msg->linear.x;// + 0.5*(2.0*(double)rand()/RAND_MAX - 1.0);
    W_in = msg->angular.z;// + 0.5*(2.0*(double)rand()/RAND_MAX - 1.0);
}

void PuzzleBotSim::SetCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    double cmd_Vx, cmd_Wz;
    cmd_Vx = msg->linear.x;
    cmd_Wz = -msg->angular.z;

    cmd_wR = (1.0 / params["Rr"]) * (cmd_Vx + cmd_Wz * params["L"] / 2);
    cmd_wL = (1.0 / params["Rl"]) * (cmd_Vx - cmd_Wz * params["L"] / 2);
}

void PuzzleBotSim::SetWrCallback(const std_msgs::Float32::ConstPtr& msg){
    cmd_wR = msg->data;
}


void PuzzleBotSim::SetWlCallback(const std_msgs::Float32::ConstPtr& msg){
    cmd_wL = msg->data;
}


void PuzzleBotSim::SetPWMrCallback(const std_msgs::Float32::ConstPtr& msg){
    cmd_pwmR = msg->data;
}


void PuzzleBotSim::SetPWMlCallback(const std_msgs::Float32::ConstPtr& msg){
    cmd_pwmL = msg->data;
}

void PuzzleBotSim::WrCallback(const std_msgs::Float32::ConstPtr& msg){
    sys_wR = msg->data;
}

void PuzzleBotSim::WlCallback(const std_msgs::Float32::ConstPtr& msg){
    sys_wL = msg->data;
}

int PuzzleBotSim::GetLoopRate(){
    return loop_rate;
}

void PuzzleBotSim::Run(){
    double vx_dot_temp,vy_dot_temp,w_dot_temp;
    double v_dot_max,w_dot_max;
    double dt=0.01;

    current_time = ros::Time::now();

    dt = (current_time - last_time).toSec();
    last_time = current_time;

    /*v_dot_max = 0.5;
    w_dot_max = 1;

    vx_dot_temp = (Vx_in - Vx)/dt;
    if(fabs(vx_dot_temp)>v_dot_max)
        vx_dot_temp = sign(vx_dot_temp)*v_dot_max;

    Vx += dt*vx_dot_temp;

    vy_dot_temp = (Vy_in - Vy)/dt;
    if(fabs(vy_dot_temp)>v_dot_max)
        vy_dot_temp = sign(vy_dot_temp)*v_dot_max;

    Vy += dt*vy_dot_temp;

    w_dot_temp = (W_in - W)/dt;
    if(fabs(w_dot_temp)>w_dot_max)
        w_dot_temp = sign(w_dot_temp)*w_dot_max;

    W += dt*w_dot_temp;

    x += (Vx*cos(theta) - Vy*sin(theta))*dt;
    y += (Vx*sin(theta) + Vy*cos(theta))*dt;
    theta += W*dt;*/

    x_n += (Vx_in*cos(theta_n))*dt;
    y_n += (Vx_in*sin(theta_n))*dt;
    theta_n += W_in*dt;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_n);
    geometry_msgs::Quaternion odom_quat_r = tf::createQuaternionMsgFromYaw(theta_n);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_n;
    odom_trans.transform.translation.y = y_n;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = Vx_in;
    odom.twist.twist.linear.y = Vy_in;
    odom.twist.twist.angular.z = W_in;

    odom.pose.pose.position.x = x_n;
    odom.pose.pose.position.y = y_n;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat_r;

    pub_odom.publish(odom);

    if(params["mode"] < 3){
        cmd_pwmR = PWMR_controller.GetControl(cmd_wR,sys_wR);
        cmd_pwmL = PWML_controller.GetControl(cmd_wL,sys_wL);
    }
    std_msgs::Float32 msgR,msgL;
    msgR.data = cmd_pwmR;
    msgL.data = cmd_pwmL;
    pub_pwmR.publish(msgR);
    pub_pwmL.publish(msgL);

    std_msgs::Float32 msgv, msgw;
    msgv.data = x_n;
    msgw.data = y_n;
    pub_v.publish(msgv);
    pub_w.publish(msgw);

}

}