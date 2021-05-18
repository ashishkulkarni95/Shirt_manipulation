//****************************************************************************************
//
// Author : Aniruddha Shembekar, University of Southern California
//
//****************************************************************************************

#ifndef GRASPING_ROBOT2_MOTION_NODE_HPP
#define GRASPING_ROBOT2_MOTION_NODE_HPP

#include <ros/ros.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <vector>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include "gen_utilities/KukaJointAnglesSet.h"
#include "gen_utilities/TrajContainerSrv.h"
#include "gen_utilities/KinectData.h"

class GraspingRob2Motion
{
public:
    ros::NodeHandle n_grasp_rob_mtn;
    ros::Publisher grasp_rob_mtn_pub;
    ros::Subscriber comm_motion_sub;
    ros::Publisher gripper_state_val_pub;
    
    ros::ServiceServer rob_layup_motion_data_serv;
    ros::ServiceServer blue_rob_grasping_mtn;
    ros::ServiceServer kinect_rob_blue_motion_data_pub;
    ros::Subscriber rob_motion_data_sub_ch2;
    ros::Subscriber rob_layup_motion_terminator_sub;
    ros::ServiceServer sheet_load_rob_blue_motion_data_pub;
    // ros::Publisher start_gripper_to_coord_pub;
    ros::Publisher gripper_rotation_pub_blue;

    sensor_msgs::JointState S;
    std::string ip;
    int port;
    robot_comm::RobotComm* grasp_rob2_comm;
    bool status;
    std::string rcvd_str;
    std::string this_rob_mtn;
    std_msgs::String loc_val;
    std_msgs::String str_val;
    std_msgs::String gripper_status;
    bool terminate_layup = false;
    std_msgs::String grasp_rob_str_loc;
    std_msgs::String rob_reply;
    int count = 0;
    std::string gui_term_sig;
    bool layup_started = false;
    bool toggle_grippers_during_regrasp;
    std::string grasp_rob2_status;
    std_msgs::Int16 vel_blue_gripp;
    bool flag_to_check_comm = false;
        
    GraspingRob2Motion(std::string ip_val, int port_val, std::string regrasp_toggle);
    ~GraspingRob2Motion();

    void getLayupMotionMsg(const std_msgs::String&);

    void getGraspRobSyncMsg(const std_msgs::String&);

    void getLayupTrajTerminationConfirmation(const std_msgs::String&);

    void send_mtn_confirmation();

    void getCommState(const std_msgs::String&);
    
    bool establish_comm();

    void close_comm();

    void getTerminationMsg(const std_msgs::String str);

    void getGUISignalMsg(const std_msgs::String&);

    void getCommMotionMsg(const std_msgs::String&);

    bool getCommLayupMotionMsg(gen_utilities::TrajContainerSrv::Request &reg,
        gen_utilities::TrajContainerSrv::Response &res);

    bool getCommGraspingMotion(gen_utilities::TrajContainerSrv::Request &reg,
    gen_utilities::TrajContainerSrv::Response &res);

    bool StrtKinectMotion(gen_utilities::KinectData::Request &req,
        gen_utilities::KinectData::Response &res);

    bool StrtSheetLoadingMotion(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res);
    
    bool start_rolling_gripper_action_blue(int vel, std::string direction);


    // void start_grippers();

};

#endif