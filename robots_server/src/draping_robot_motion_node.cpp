//****************************************************************************************
//
// Author : Aniruddha Shembekar, Jaineel Desai -  University of Southern California
//
//****************************************************************************************

#include <ros/ros.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <vector>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include "gen_utilities/robot_comm.hpp"
#include "gen_utilities/file_rw.hpp"
#include "robots_server/draping_robot_motion_node.hpp"
#include "gen_utilities/TrajContainer.h"
#include "gen_utilities/rosmsg_eigen_exchange.hpp"
#include "gen_utilities/TrajContainerSrv.h"
#include "gen_utilities/RobotHalt.h"

DrapingRobMotion::DrapingRobMotion(std::string ip_val, int port_val)
{
    drape_rob_mtn_pub = n_drape_rob_mtn.advertise<sensor_msgs::JointState>("drape_rob_state", 1000);
    drape_rob_layup_mtn_pub = n_drape_rob_mtn.advertise<std_msgs::String>("coordinator_layup_mtn_rob_reply", 1000);
    comm_confirmation_pub = n_drape_rob_mtn.advertise<std_msgs::String>("comm_confirmation_pub", 1000);
    comm_motion_sub = n_drape_rob_mtn.subscribe("rob_comm_state", 1000, &DrapingRobMotion::getCommState, this);
    
    // rob motions instructions through services
    rob_probe_motion_data_serv = n_drape_rob_mtn.advertiseService("drape_rob_comm/probe_motion", &DrapingRobMotion::getCommProbeMotionMsg, this); 
    rob_layup_motion_data_serv = n_drape_rob_mtn.advertiseService("drape_rob_comm/layup_motion", &DrapingRobMotion::getCommLayupMotionMsg, this); 

    coodinator_msgs_to_gui_pub = n_drape_rob_mtn.advertise<std_msgs::String>("coodinator_msgs_to_gui", 1000);     
    rob_motion_data_sub_ch2 = n_drape_rob_mtn.subscribe("gui_rob_ctrl", 1000, &DrapingRobMotion::getGUISignalMsg, this);
	halt_rob = n_drape_rob_mtn.serviceClient<gen_utilities::RobotHalt>("drape_rob_comm/halt_motion");
    


    ip = ip_val;
    port = port_val;
    drape_rob_comm = new robot_comm::RobotComm(ip,port,"server");
    gui_term_sig = "no_terminate";
};

void DrapingRobMotion::getCommState(const std_msgs::String& str)
{
    if (str.data.compare("START")==0 || str.data.compare("START_DRAPE_ONLY")==0)
    {
        status = true;
        std_msgs::String str;
        if(flag_to_check_comm == false){
            if(!establish_comm())
            {
                ROS_INFO("Connection could not be established for draping robot motion");    
                str.data = "not_connected";
                comm_confirmation_pub.publish(str);
            }
            else
            {
                flag_to_check_comm = true;
                str.data = "connected";
                comm_confirmation_pub.publish(str);
            }
        }
    }
    if (str.data.compare("STOP")==0)
    {
        close_comm();
        status = false;
        flag_to_check_comm = false;
    }
};

bool DrapingRobMotion::establish_comm()
{
    return drape_rob_comm->establishComm();
};

void DrapingRobMotion::close_comm()
{
    drape_rob_comm->sendString("terminate_connection");
    drape_rob_comm->closeComm();   
} 

void DrapingRobMotion::getGUISignalMsg(const std_msgs::String& str)
{
    gui_term_sig = str.data;
}

bool DrapingRobMotion::getCommProbeMotionMsg(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{
    // checking if the user pressed terminate probing
    if (gui_term_sig.compare("terminate_probing")==0)
    {
        Eigen::MatrixXd terminator_mat = Eigen::MatrixXd::Zero(1,1);
        drape_rob_comm->sendDoubleMatrix(terminator_mat);
        rcvd_str = drape_rob_comm->receiveString();  // receive the terminating confirmation from robot
        probing_started = false;
        gui_term_sig = "no_terminate";
        return false;
    }
    // making decision if probing already started or not
    if (!probing_started)
    {
        drape_rob_comm->sendString("execute_probing");
        rcvd_str = drape_rob_comm->receiveString();         // read conformation to start : "start_probing"
        if (rcvd_str.compare("start_probing")==0)
        {
            probing_started = true;
        }
    } 
    Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
    Eigen::MatrixXd grp_idx = ros_eigen_exch::group_idx_to_eigen(req.J);
    Eigen::MatrixXi grp_idx_casted = grp_idx.cast<int>();
    Eigen::MatrixXd xyz_cba = ros_eigen_exch::xyz_cba_to_eigen(req.WP);
    drape_rob_comm->sendDoubleMatrix(joint_config);
    rcvd_str = drape_rob_comm->receiveString();         // read conformation to send joint angles : "angles_rcvd"
    drape_rob_comm->sendIntMatrix(grp_idx_casted);
    rcvd_str = drape_rob_comm->receiveString();         // read conformation to send grp index : "grp_idx_rcvd"
    drape_rob_comm->sendDoubleMatrix(xyz_cba);
    rcvd_str = drape_rob_comm->receiveString();         // read conformation to send xyz cba : "xyz_cba_rcvd"
    drape_rob_comm->sendString(req.status);
    if(drape_rob_comm->receiveString().compare("okay_terminating")==0)
    {
        probing_started = false;
    }
    return true;
}


bool DrapingRobMotion::getCommLayupMotionMsg(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{
    // making decision if draping already started or not
    std::cout<<"In service for starting draping robot motion........................... \n"; 

    drape_rob_comm->sendString(req.status);
    std::cout<<"Message Sent: execute_layup........................... \n";

    rcvd_str = drape_rob_comm->receiveString();
    std::cout<<"Message Received: "<<rcvd_str<<"........................... \n";

    Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
    std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
    drape_rob_comm->sendDoubleMatrix(joint_config);
    std::cout<<"Message Sent: Double Matrix........................... \n";

    rcvd_str = drape_rob_comm->receiveString();         // read conformation to send joint angles : "angles_rcvd"    
    std::cout<<"Message Received: "<<rcvd_str<<"........................... \n";    

    Eigen::MatrixXd xyz_cba = ros_eigen_exch::xyz_cba_to_eigen(req.WP);
    std::cout<<"xyz cba to be sent are...............................: \n"<<xyz_cba<<std::endl;
    drape_rob_comm->sendDoubleMatrix(xyz_cba);
    std::cout<<"Message Sent: Double Matrix........................... \n";

    rcvd_str = drape_rob_comm->receiveString();         // read conformation to send xyz cba : "xyz_cba_rcvd"
    std::cout<<"Message Received: "<<rcvd_str<<"........................... \n";

    std::cout<<"toolname to be sent are...............................: \n"<<req.tool_name<<std::endl;
    drape_rob_comm->sendString(req.tool_name);
    std::cout<<"Message Sent: tool_name..........................."<<std::endl;

    rcvd_str = drape_rob_comm->receiveString();         // read conformation to send tool name : "tool_name_revd"
    std::cout<<"Message Received: "<<rcvd_str<<"........................... \n";

    Eigen::MatrixXd joint_vel = ros_eigen_exch::velocity_to_eigen(req.J);
    std::cout<<"Joint Velocity to be sent are...............................: \n"<<joint_vel<<std::endl;
    drape_rob_comm->sendDoubleMatrix(joint_vel);
    std::cout<<"Message Sent: Double Matrix........................... \n";

    rcvd_str = drape_rob_comm->receiveString();        // read conformation to send tool name : "rcvd_vel"
    std::cout<<"Message Received: "<<rcvd_str<<"........................... \n";

    Eigen::MatrixXd group_idx = ros_eigen_exch::group_idx_to_eigen(req.J);
    std::cout<<"Group idx to be sent are...............................: \n"<<group_idx<<std::endl;
    drape_rob_comm->sendDoubleMatrix(group_idx);
    std::cout<<"Message Sent: Double Matrix........................... \n";

    rcvd_str = drape_rob_comm->receiveString();        // read conformation to send tool name : "rcvd_grp_id"
    std::cout<<"Message Received: "<<rcvd_str<<"........................... \n";

    return true;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Draping_Robot_Motion");
    std::string ip_val;
    int port_val;
    if (argc==3)
    {
        ip_val = argv[1];
        
        std::string s = argv[2];
        port_val = std::stoi(s);
    }

    DrapingRobMotion DRMObj(ip_val, port_val);
    while(ros::ok())
    {
        ros::Rate loop_rate(1000);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    DRMObj.close_comm();
    return 0;
} 