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
#include "robots_server/grasping_robot2_motion_syncer_node.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iostream>

GraspingRob2Syncer::GraspingRob2Syncer(std::string ip_val, int port_val)
{
    comm_motion_sub = n_grasp_rob2_sync.subscribe("rob_comm_state", 1000, &GraspingRob2Syncer::establish_comm, this);    
    rob_kinect_mtn = n_grasp_rob2_sync.advertiseService("kinect/blue_rob_motion", &GraspingRob2Syncer::kinect_rob_mtn, this);

    ip = ip_val;
    port = port_val;
    grasp_rob2_comm_sync = new robot_comm::RobotComm(ip,port,"server");
};

GraspingRob2Syncer::~GraspingRob2Syncer()
{

    grasp_rob2_comm_sync->closeComm();

}


void GraspingRob2Syncer::establish_comm(const std_msgs::String& str)
{
    std::cout<<"Values of flag is: Grasp Rob2 Syncer Inside func "<<flag_to_check_comm<<std::endl;
    if(str.data.compare("START") == 0)
    {
        std::cout<<"Values of flag is: Grasp Rob2 Syncer Inside start "<<flag_to_check_comm<<std::endl;
        if(flag_to_check_comm == false){
            grasp_rob2_comm_sync->establishComm();
            flag_to_check_comm = true;
            std::cout<<"Values of flag is: Grasp Rob2 Syncer after true "<<flag_to_check_comm<<std::endl;
        }
        else{
            ROS_INFO("Communication already established");
        }
    }
    if (str.data.compare("STOP")==0)
    {
        close_comm();
        //status = false;
        flag_to_check_comm = false;
        std::cout<<"Values of flag is: Grasp Rob1 Syncer Inside stop "<<flag_to_check_comm<<std::endl;
    }
    std::cout<<"Values of flag is: Grasp Rob2 Syncer after func "<<flag_to_check_comm<<std::endl;
};

bool GraspingRob2Syncer::kinect_rob_mtn(gen_utilities::SyncerNodeKinect::Request &req,
    gen_utilities::SyncerNodeKinect::Response &res)
{
    if(req.task == "start_tracking"){

        std::cout<<"Before sending start tracking \n";
        grasp_rob2_comm_sync->sendString("start_tracking");
        
        // std::cout<<"Before sending double data \n";
        // grasp_rob2_comm_sync->sendDouble(req.delta);

        std::cout<<"Service called for blue \n";
        return true;    
    }
    else
    {
        // std::cout<<"Before sending stop tracking \n";
        // grasp_rob2_comm_sync->sendString("stop_tracking");
        std::cout<<"Service called for blue to stop \n";
        return true;
    }
    return true;
}


void GraspingRob2Syncer::close_comm()
{
    grasp_rob2_comm_sync->sendString("terminate_connection");
    grasp_rob2_comm_sync->closeComm();   
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Grasping_Robot_2_Motion_Syncer");
    std::string ip_val;
    int port_val;
    if (argc==3)
    {
        ip_val = argv[1];
        
        std::string s = argv[2];
        port_val = std::stoi(s);
    }

    GraspingRob2Syncer GRSObj(ip_val, port_val);
    
    while(ros::ok())
    {
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //GRSObj.close_comm();
    return 0;
} 