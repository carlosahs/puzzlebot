#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include "puzzlebot_examples/PIDController.hpp"
#include "puzzlebot_examples/Shared.hpp"
#include <tf/tf.h>

double x,y,theta,wl,wr;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
void wr_callback(const std_msgs::Float32::ConstPtr& msg);
void wl_callback(const std_msgs::Float32::ConstPtr& msg);

int main(int arc, char ** argv)
{
    ROS_INFO("[PuzzleBot] Starting square example");
    ros::init(arc,argv,"puzzlebot_example");

    ros::Time cur_time,last_time;

    x = -0.8;//default values to match gazebo
    y = -0.8;
    theta = 0;
    wr = 0;
    wl = 0;

    ros::NodeHandle nh;
    std::string prefix = "";
    if (arc > 1)
        prefix = argv[1];
    
    
    std::map<std::string,double> params;
    std::map<std::string,std::string> topics;
    params = PuzzleBotSimulator::FetchParameters("/distance_controller",nh);
    topics = PuzzleBotSimulator::FetchTopics("/distance_controller",nh);

    PuzzleBotSimulator::PIDController dist_ctrl,ang_ctrl;
    dist_ctrl.SetParameters(params["Kp_dist"],params["Ti_dist"],params["Td_dist"]);
    dist_ctrl.SetControlLimits(params["v_min"],params["v_max"]);
    ang_ctrl.SetParameters(params["Kp_ang"],params["Ti_ang"],params["Td_ang"]);
    ang_ctrl.SetControlLimits(params["w_min"],params["w_max"]);

    double points[2][4] = { {-0.8,0.8,0.8,-0.8},
                            {-0.8,-0.8,0.8,0.8}   };
    
    std::string cmd_vel_topic, odom_topic, simulator,wr_topic,wl_topic;
    ros::Subscriber sub_odom,sub_wr,sub_wl;
    if(!nh.getParam("/distance_controller/simulator",simulator)){

        ROS_ERROR("[PuzzleBot] Please specify a simulator %s",simulator.c_str());
        ros::shutdown();
    } else if(simulator == "rviz"){
        cmd_vel_topic = topics["cmd_vel_rviz"];
        odom_topic = topics["odom_rviz"];
        sub_odom = nh.subscribe(odom_topic,10,&odom_callback);
    } else if(simulator == "gazebo"){
        cmd_vel_topic = topics["cmd_vel_gazebo"];
        wr_topic = topics["wr_gazebo"];
        wl_topic = topics["wl_gazebo"];
        sub_wr = nh.subscribe(wr_topic,10,&wr_callback);
        sub_wl = nh.subscribe(wl_topic,10,&wl_callback);
    } else if (simulator == "none"){
        cmd_vel_topic = topics["cmd_vel_robot"];
        wr_topic = topics["wr_robot"];
        wl_topic = topics["wl_robot"];
        sub_wr = nh.subscribe(wr_topic,10,&wr_callback);
        sub_wl = nh.subscribe(wl_topic,10,&wl_callback);
    } else {
        ROS_ERROR("Invalid simulator provided. Supported options are 'rviz','gazebo','none'");
        ros::shutdown();
    }
    
    //ros::Subscriber sub_odom = nh.subscribe(odom_topic,10,&odom_callback);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic,100);

    int i = 0;
    ros::Rate loop_rate (100);
    while(ros::ok()){

        ros::spinOnce();
        if(simulator == "none" || simulator == "gazebo"){
            cur_time = ros::Time::now();
            double dt = (cur_time - last_time).toSec();
            double v = (0.05 * wr + 0.05 * wl)/2.0;
            double w = (0.05 * wr - 0.05 * wl)/0.191;
            x += v * dt * cos(theta);
            y += v * dt * sin(theta);
            theta += w * dt;
            last_time = cur_time;
        }
        double e_x,e_y,error_dist,error_ang;
        e_x = points[0][i] - x;
        e_y = points[1][i] - y;
        error_dist = sqrt(pow(e_x,2) + pow(e_y,2));
        error_ang = PuzzleBotSimulator::WrapToPi(atan2(e_y,e_x) - theta);
        double v = - dist_ctrl.GetControl(0,error_dist);
        double w = - ang_ctrl.GetControl(0,error_ang);
        geometry_msgs::Twist msg;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;

        msg.linear.x = v;
        msg.angular.z = w;

        pub_cmd_vel.publish(msg);
        if(error_dist < 0.05){
            if(i < 4)
                i++;
            else
                i = 0;
        }
        loop_rate.sleep();
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    theta = yaw;
}

void wr_callback(const std_msgs::Float32::ConstPtr& msg){
    wr = msg->data;
}
void wl_callback(const std_msgs::Float32::ConstPtr& msg){
    wl = msg->data;
}