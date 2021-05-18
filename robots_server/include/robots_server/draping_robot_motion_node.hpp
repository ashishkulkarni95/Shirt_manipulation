//****************************************************************************************
//
// Author : Aniruddha Shembekar, University of Southern California
//
//****************************************************************************************

#ifndef DRAPING_ROBOT_MOTION_NODE_HPP
#define DRAPING_ROBOT_MOTION_NODE_HPP

#include <ros/ros.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <vector>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <iostream>
#include "gen_utilities/TrajContainer.h"
#include "gen_utilities/TrajContainerSrv.h"


class DrapingRobMotion
{
public:
    ros::NodeHandle n_drape_rob_mtn;
    ros::Publisher drape_rob_mtn_pub;
    ros::Subscriber comm_motion_sub;
    ros::Subscriber drape_rob_layup_mtn_sub;
    ros::Publisher drape_rob_layup_mtn_pub;
    ros::Subscriber rob_motion_data_sub;
    ros::Publisher comm_confirmation_pub;
    ros::Subscriber layup_traj_terminate_msg_sub;
    ros::Publisher coodinator_msgs_to_gui_pub;
    ros::Subscriber rob_motion_data_sub_ch2;
    ros::ServiceServer rob_probe_motion_data_serv;
    ros::ServiceServer rob_layup_motion_data_serv;
    ros::ServiceClient halt_rob;

    
    sensor_msgs::JointState S;
    std::string ip;
    int port;
    robot_comm::RobotComm* drape_rob_comm;
    bool status;
    bool terminate_layup = false;
    std::string rcvd_str;

    std_msgs::String loc_val;
    std_msgs::String reply;
    gen_utilities::TrajContainer drape_traj_container;

    std::string gui_term_sig;
    bool probing_started = false;
    bool layup_started = false;

    bool flag_to_check_comm = false;
        
    DrapingRobMotion(std::string ip_val, int port_val);

    void getLayupMotionMsg(const std_msgs::String&);

    void getLayupTrajTerminationConfirmation(const std_msgs::String&);
    
    void getCommState(const std_msgs::String&);
    
    bool establish_comm();

    void close_comm();
    
    void getGUISignalMsg(const std_msgs::String&);

    void updateTrajectory(const gen_utilities::TrajContainer &traj_data);

    bool getCommProbeMotionMsg(gen_utilities::TrajContainerSrv::Request &reg,
        gen_utilities::TrajContainerSrv::Response &res);

    bool getCommLayupMotionMsg(gen_utilities::TrajContainerSrv::Request &reg,
        gen_utilities::TrajContainerSrv::Response &res);

};

#endif