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
#include "robots_server/draping_robot_state_node.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iostream>
#include <Eigen/Eigen>
#include "gen_utilities/rosmsg_eigen_exchange.hpp"

DrapingRobState::DrapingRobState(std::string ip_val, int port_val)
{
    drape_rob_state_pub = n_drape_rob.advertise<sensor_msgs::JointState>("drape_rob_state", 1000);
    comm_state_sub = n_drape_rob.subscribe("rob_comm_state", 1000, &DrapingRobState::getCommState, this);
    ip = ip_val;
    port = port_val;
    rc_d = new robot_comm::RobotComm(ip,port,"server");
};

void DrapingRobState::getCommState(const std_msgs::String& str)
{
    if (str.data.compare("STOP")==0)
    {
        status = false;
    }
	if (status) return;
    if (str.data.compare("START")==0)
    {
        status = true;
        start_comm();
    }
};

void DrapingRobState::start_comm()
{
    if(rc_d->establishComm())
    {
        while (ros::ok() && status)
        {
            rc_d->sendString("next");
            Eigen::MatrixXd data = rc_d->receiveDouble1DMat();            
            S = ros_eigen_exch::eigen_to_joint_state(data);
            drape_rob_state_pub.publish(S);
            ros::Rate loop_rate(10);
            ros::spinOnce();
            loop_rate.sleep();
        }
        rc_d->sendString("cancel");
    }    
    rc_d->closeComm();   
};

void DrapingRobState::close_comm()
{
    rc_d->sendString("terminate_connection");
    rc_d->closeComm();   
}; 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Draping_Robot_State");

    std::string ip_val;
    int port_val;
    if (argc==3)
    {
        ip_val = argv[1];
        
        std::string s = argv[2];
        port_val = std::stoi(s);
    }

    DrapingRobState DRSObj(ip_val, port_val);
    
    while(ros::ok())
    {
        ros::Rate loop_rate(5);
        ros::spinOnce();
        loop_rate.sleep();
    }

    DRSObj.close_comm();

    return 0;
} 