#ifndef GRASPING_ROBOT2_SYNCER_NODE
#define GRASPING_ROBOT2_SYNCER_NODE

#include <ros/ros.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <vector>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iostream>
#include "gen_utilities/SyncerNodeKinect.h"

class GraspingRob2Syncer
{
public:
    ros::NodeHandle n_grasp_rob2_sync;
    ros::Subscriber comm_motion_sub;
    //ros::Publisher grasp_motion_finished_teller;
    ros::Subscriber kinect_grasp_motion_planner;
    ros::ServiceServer rob_kinect_mtn;
    
    std::string ip;
    int port;
    robot_comm::RobotComm* grasp_rob2_comm_sync;
    std::string rcvd_str;
    bool flag_to_check_comm = false;
            
    GraspingRob2Syncer(std::string ip_val, int port_val);

    ~GraspingRob2Syncer();

    //void getCommState(const std_msgs::String&);

    void kinect_motions(const std_msgs::String &);
    
    void establish_comm(const std_msgs::String&);

    void close_comm();

    bool kinect_rob_mtn(gen_utilities::SyncerNodeKinect::Request &req,
        gen_utilities::SyncerNodeKinect::Response &res);


};

#endif