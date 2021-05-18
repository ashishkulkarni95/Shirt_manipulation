//****************************************************************************************
//
// Author : Aniruddha Shembekar, University of Southern California
//
//****************************************************************************************

#ifndef GRASPING_ROBOT1_STATE_NODE_HPP
#define GRASPING_ROBOT1_STATE_NODE_HPP

#include <ros/ros.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <vector>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iostream>

class GraspingRob1State
{
public:
    ros::NodeHandle n_grasp_rob;
    ros::Publisher grasp_rob_state_pub;
    ros::Subscriber comm_state_sub;
    
    sensor_msgs::JointState S;
    std::string ip;
    int port;
    bool status = false;
    
    robot_comm::RobotComm* rc_g1;
    
    GraspingRob1State(std::string ip_val, int port_val);

    void getCommState(const std_msgs::String& );

    void start_comm();

    void close_comm();

};

#endif