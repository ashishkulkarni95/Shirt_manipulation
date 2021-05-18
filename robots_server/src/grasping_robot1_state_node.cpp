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
#include "robots_server/grasping_robot1_state_node.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iostream>
#include "gen_utilities/rosmsg_eigen_exchange.hpp"

GraspingRob1State::GraspingRob1State(std::string ip_val, int port_val)
{
    grasp_rob_state_pub = n_grasp_rob.advertise<sensor_msgs::JointState>("grasp_rob_1_state", 1000);
    comm_state_sub = n_grasp_rob.subscribe("rob_comm_state", 1000, &GraspingRob1State::getCommState, this);
    ip = ip_val;
    port = port_val;
    rc_g1 = new robot_comm::RobotComm(ip,port,"server");
};

void GraspingRob1State::getCommState(const std_msgs::String& str)
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

void GraspingRob1State::start_comm()
{
    if(rc_g1->establishComm())
    { 
        while (ros::ok() && status)
        {
            rc_g1->sendString("next");
            Eigen::MatrixXd data = rc_g1->receiveDouble1DMat();
            S = ros_eigen_exch::eigen_to_joint_state(data);
            grasp_rob_state_pub.publish(S);
            ros::Rate loop_rate(10);
            ros::spinOnce();
            loop_rate.sleep();
            // std::cout << status << std::endl;
        }
        // std::cout << status << std::endl;
        
        rc_g1->sendString("cancel");
    }
    rc_g1->closeComm();   
};

void GraspingRob1State::close_comm()
{
    rc_g1->sendString("terminate_connection");
    rc_g1->closeComm();   
}; 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Grasping_Robot_1_State");

    std::string ip_val;
    int port_val;
    if (argc==3)
    {
        ip_val = argv[1];
        
        std::string s = argv[2];
        port_val = std::stoi(s);

    }

    GraspingRob1State GRSObj(ip_val, port_val);
    
    while(ros::ok())
    {
        ros::Rate loop_rate(5);
        ros::spinOnce();
        loop_rate.sleep();
    }

    GRSObj.close_comm();

    return 0;
} 