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
#include "gen_utilities/robot_comm.hpp"
#include "robots_server/grasping_robot1_motion_syncer_node.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iostream>

GraspingRob1Syncer::GraspingRob1Syncer(std::string ip_val, int port_val)
{
    comm_motion_sub = n_grasp_rob_sync.subscribe("rob_comm_state", 1000, &GraspingRob1Syncer::establish_comm, this);
    grasp_motion_finished_teller = n_grasp_rob_sync.advertise<std_msgs::String>("grasp_rob1_comm/motion_finished", 1000);
    rob_kinect_mtn = n_grasp_rob_sync.advertiseService("kinect/green_rob_motion", &GraspingRob1Syncer::kinect_rob_mtn, this);

    ip = ip_val;
    port = port_val;
    grasp_rob1_comm_sync = new robot_comm::RobotComm(ip,port,"server");
};

GraspingRob1Syncer::~GraspingRob1Syncer()
{
    grasp_rob1_comm_sync->closeComm();
}


void GraspingRob1Syncer::establish_comm(const std_msgs::String& str)
{   std::cout<<"Values of flag is: Grasp Rob1 Syncer Inside function "<<flag_to_check_comm<<std::endl;
    if(str.data.compare("START") == 0)
    {
        std::cout<<"Values of flag is: Grasp Rob1 Syncer Inside start "<<flag_to_check_comm<<std::endl;
        if(flag_to_check_comm == false){
            grasp_rob1_comm_sync->establishComm();
            flag_to_check_comm = true;
        }
        else{
            ROS_INFO("Communicatio already established");
        }
    }
    if (str.data.compare("STOP")==0)
    {
        close_comm();
        //status = false;
        flag_to_check_comm = false;
        std::cout<<"Values of flag is: Grasp Rob1 Syncer Inside stop "<<flag_to_check_comm<<std::endl;
    }
};


bool GraspingRob1Syncer::kinect_rob_mtn(gen_utilities::SyncerNodeKinect::Request &req,
    gen_utilities::SyncerNodeKinect::Response &res)
{
    if(req.task == "start_tracking"){
        std::cout<<"Before sending start tracking \n";
        grasp_rob1_comm_sync->sendString("start_tracking");

        // std::cout<<"Before sending double data \n";
        // grasp_rob1_comm_sync->sendDouble(req.delta);

        std::cout<<"Service called for green \n";
        return true;    
    }
    else
    {
        // std::cout<<"Before sending stop tracking \n";
        // grasp_rob1_comm_sync->sendString("stop_tracking");
        std::cout<<"Service called for green to stop \n";
        return true;
    }
    return true;
}

void GraspingRob1Syncer::close_comm()
{
    grasp_rob1_comm_sync->sendString("terminate_connection");
    grasp_rob1_comm_sync->closeComm();   
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Grasping_Robot_1_Motion_Syncer");
    std::string ip_val;
    int port_val;
    if (argc==3)
    {
        ip_val = argv[1];
        
        std::string s = argv[2];
        port_val = std::stoi(s);
    }

    GraspingRob1Syncer GRSObj(ip_val, port_val);
    
    while(ros::ok())
    {
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
    // GRSObj.close_comm();
    return 0;
} 