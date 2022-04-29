#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/JointControllerState.h>
//hi
/*void wr_real_callback(const std_msgs::Float32::ConstPtr& msg);
void wl_real_callback(const std_msgs::Float32::ConstPtr& msg);
void wr_sim_callback(const control_msgs::JointControllerState::ConstPtr& msg);
void wl_sim_callback(const control_msgs::JointControllerState::ConstPtr& msg);*/

double wr_real,wl_real,wr_sim,wl_sim;
int main(int arc, char ** argv)
{
    ROS_INFO("[PuzzleBot] Starting test");
    ros::init(arc,argv,"test_motor_constants");

    ros::NodeHandle nh;

    ros::Publisher p_sim_r,p_sim_l;
    ros::Publisher p_real_r,p_real_l;

    ros::Subscriber sub_sim_wr,sub_sim_wl;
    ros::Subscriber sub_real_wr,sub_real_wl;

    /*sub_real_wr = nh.subscribe("/wr",10,wr_real_callback);
    sub_real_wl = nh.subscribe("/wl",10,wl_real_callback);
    sub_sim_wr = nh.subscribe("/right_wheel_velocity_controller/state",10,wr_sim_callback);
    sub_sim_wl = nh.subscribe("/left_wheel_velocity_controller/state",10,wl_sim_callback);*/

    p_real_r = nh.advertise<std_msgs::Float32>("/cmd_wR",100);
    p_real_l = nh.advertise<std_msgs::Float32>("/cmd_wL",100);
    p_sim_r = nh.advertise<std_msgs::Float64>("/right_wheel_velocity_controller/command",100);
    p_sim_l = nh.advertise<std_msgs::Float64>("/left_wheel_velocity_controller/command",100);

    ros::Rate loop_rate(15);

    std::vector<double> pwms(201);

    /*for(int i = 0; i < 50; i++)
	pwms[i] = 0.2;
    for(int i = 50; i<100; i++)
	pwms[i] = 0.0;
    for(int i = 100; i < 150; i++)
	pwms[i] = -0.2;
    for(int i = 150; i < 201; i++)
	pwms[i] = 0.0;*/

    for(int i = 0; i < 201; i++){
        pwms[i] = 10.0 * sin(((double)(i - 100.0) / 100.0) * M_PI);
    }
    geometry_msgs::Twist msg;
    while(ros::ok()){
        for(int i = 0; i < 201;i++){
            std_msgs::Float64 msg64;
            std_msgs::Float32 msg32;
            msg64.data = pwms[i];
            msg32.data = pwms[i];
            p_sim_l.publish(msg64);
            p_sim_r.publish(msg64);
            p_real_l.publish(msg32);
            p_real_r.publish(msg32);

            ros::spinOnce();

            loop_rate.sleep();
        }
        
    }
    
}
/*
void wr_real_callback(const std_msgs::Float32::ConstPtr& msg){
    wr_real = msg->data;
}
void wl_real_callback(const std_msgs::Float32::ConstPtr& msg){
    wl_real = msg->data;
}

void wr_sim_callback(const control_msgs::JointControllerState::ConstPtr& msg){
    wr_sim = msg->process_value;
}

void wr_sim_callback(const control_msgs::JointControllerState::ConstPtr& msg){
    wr_sim = msg->process_value;
}*/