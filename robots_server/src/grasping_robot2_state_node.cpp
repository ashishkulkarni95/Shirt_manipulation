//****************************************************************************************
//
// Author : Aniruddha Shembekar, University of Southern California
//
//****************************************************************************************

#include <ros/ros.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <vector>
#include "gen_utilities/robot_comm.hpp"
#include "robots_server/grasping_robot2_state_node.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iostream>
#include "gen_utilities/rosmsg_eigen_exchange.hpp"

GraspingRob2State::GraspingRob2State(std::string ip_val, int port_val)
{
    grasp_rob_state_pub = n_grasp_rob.advertise<sensor_msgs::JointState>("grasp_rob_2_state", 1000);
    comm_state_sub = n_grasp_rob.subscribe("rob_comm_state", 1000, &GraspingRob2State::getCommState, this);
    ip = ip_val;
    port = port_val;
    rc_g2 = new robot_comm::RobotComm(ip,port,"server");
};

void GraspingRob2State::getCommState(const std_msgs::String& str)
{
    if (str.data.compare("STOP")==0)
    {
        status = false;
    }
    if (status) return;
    // std::cout << str.data << std::endl;
    if (str.data.compare("START")==0)
    {
        status = true;
        start_comm();
    }
};

void GraspingRob2State::start_comm()
{
    if(rc_g2->establishComm())
    { 
        while (ros::ok() && status)
        {
            rc_g2->sendString("next");
            Eigen::MatrixXd data = rc_g2->receiveDouble1DMat();
            S = ros_eigen_exch::eigen_to_joint_state(data);
            grasp_rob_state_pub.publish(S);
            ros::Rate loop_rate(10);
            ros::spinOnce();
            loop_rate.sleep();
            // std::cout << status << std::endl;
        }
        // std::cout << status << std::endl;
        
        rc_g2->sendString("cancel");
    }
    rc_g2->closeComm();   
};

void GraspingRob2State::close_comm()
{
    rc_g2->sendString("terminate_connection");
    rc_g2->closeComm();   
}; 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Grasping_Robot_2_State");

    std::string ip_val;
    int port_val;
    if (argc==3)
    {
        ip_val = argv[1];
        
        std::string s = argv[2];
        port_val = std::stoi(s);

    }

    GraspingRob2State GRSObj(ip_val, port_val);
    
    while(ros::ok())
    {
        ros::Rate loop_rate(5);
        ros::spinOnce();
        loop_rate.sleep();
    }

    GRSObj.close_comm();

    return 0;
} 