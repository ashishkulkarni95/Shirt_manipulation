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
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <iostream>

#include "gen_utilities/robot_comm.hpp"
#include "robots_server/grasping_robot1_motion_node.hpp"
#include "gen_utilities/rosmsg_eigen_exchange.hpp"
#include "gen_utilities/KukaJointAnglesSet.h"
#include "gen_utilities/TrajContainerSrv.h"
#include "gen_utilities/KinectData.h"

GraspingRob1Motion::GraspingRob1Motion(std::string ip_val, int port_val, std::string regrasp_toggle)
{
    grasp_rob_mtn_pub = n_grasp_rob_mtn.advertise<sensor_msgs::JointState>("grasp_rob1_state", 1000);
    gripper_state_val_pub = n_grasp_rob_mtn.advertise<std_msgs::String>("gripper_state_val", 1000);
    comm_motion_sub = n_grasp_rob_mtn.subscribe("rob_comm_state", 1000, &GraspingRob1Motion::getCommState, this);
    rob_layup_motion_data_serv = n_grasp_rob_mtn.advertiseService("grasp_rob1_comm/layup_motion", &GraspingRob1Motion::getCommLayupMotionMsg, this); 
    rob_motion_data_sub2 = n_grasp_rob_mtn.subscribe("gui_rob_ctrl", 1000, &GraspingRob1Motion::getGUISignalMsg, this);
    rob_layup_motion_terminator_sub = n_grasp_rob_mtn.subscribe("grasp_rob_comm/terminator", 1000, &GraspingRob1Motion::getTerminationMsg, this); 
    grasp_rob_sync = n_grasp_rob_mtn.advertiseService("grasp_rob1_comm/sync_motion", &GraspingRob1Motion::checkMotionSynced, this); 
    grasp_rob1_syncer_sub = n_grasp_rob_mtn.subscribe("grasp_rob1_comm/motion_finished", 1000, &GraspingRob1Motion::checkIfMotionFinished, this);
    grasp_rob_green_motion_pub = n_grasp_rob_mtn.advertiseService("grasp_rob1_comm/grasping_motion", &GraspingRob1Motion::getCommTrajMsg, this);
    kinect_rob_green_motion_data_pub = n_grasp_rob_mtn.advertiseService("grasp_rob1_comm/kinect_motion", &GraspingRob1Motion::StrtKinectMotion, this);
    sheet_load_rob_green_motion_data_pub = n_grasp_rob_mtn.advertiseService("grasp_rob1_comm/sheet_loading_mtn", &GraspingRob1Motion::StrtSheetLoadingMotion, this);
    gripper_rotation_pub_green = n_grasp_rob_mtn.advertise<std_msgs::Int16>("robot2_speed_mmps",1000);
    
    

    ip = ip_val;
    port = port_val;
    grasp_rob1_comm = new robot_comm::RobotComm(ip,port,"server");
    str_val.data = "motion";
    gui_term_sig = "no_terminate";
    goal_reached = false;
    grasp_rob1_status = " ";

    if (regrasp_toggle.compare("yes")==0)
    {
        toggle_grippers_during_regrasp = true;        
    }
    else
    {
        toggle_grippers_during_regrasp = false;            
    }
};

GraspingRob1Motion::~GraspingRob1Motion()
{
    std::cout<<"Destructor for green robot called \n";
}

void GraspingRob1Motion::getGUISignalMsg(const std_msgs::String& str)
{
    gui_term_sig = str.data;
};

void GraspingRob1Motion::getTerminationMsg(const std_msgs::String str)
{
    if (str.data.compare("terminate_layup")==0)
    {
        Eigen::MatrixXd terminator_mat = Eigen::MatrixXd::Zero(1,1);
        grasp_rob1_comm->sendDoubleMatrix(terminator_mat);
        rcvd_str = grasp_rob1_comm->receiveString();  // receive the terminating confirmation from robot
        layup_started = false;
        gui_term_sig = "no_terminate";
    }
};

bool GraspingRob1Motion::StrtKinectMotion(gen_utilities::KinectData::Request &req,
        gen_utilities::KinectData::Response &res)
{
    if (req.status.compare("start_kinect") == 0)
    {
        grasp_rob1_comm->sendString("place_sheet");
    }
    else if(req.status.compare("moving_robots") == 0)
    {
        //grasp_rob1_comm->sendString("start_tracking");
        grasp_rob1_comm->sendDouble(req.distance);
    }
    else if(req.status.compare("stop_robots") == 0)
    {
        grasp_rob1_comm->sendString("");
    } 
}

bool GraspingRob1Motion::start_rolling_gripper_action_green(int vel, std::string direction)
{
    //vel = 200;
    if(direction == "negative"){
        vel = vel * (-1);
    }
    vel_green_gripp.data = vel;
    std::cout<<"Before sending message for gripper_ctrl............................ \n";
    gripper_rotation_pub_green.publish(vel_green_gripp);
    std::cout<<"Message to gripper published \n";
}


bool GraspingRob1Motion::StrtSheetLoadingMotion(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{
    std::cout<<"In service for executing green robot motion for sheet loading........................... \n";
    grasp_rob1_comm->sendString("perform_sheet_loading");
    std::cout<<"Message Sent: perform_sheet_loading........................... \n";

    grasp_rob1_status = grasp_rob1_comm->receiveString(); //completed sheet loading
    std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

    return true;  
}

bool GraspingRob1Motion::getCommTrajMsg(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{
    std::cout<<"In service for starting green robot motion........................... \n";
    if(req.status.compare("initial_layup_pos") == 0)
    {
        grasp_rob1_comm->sendString("initialize_grasp");
        std::cout<<"Message Sent: initialize_grasp........................... \n";

        std::cout<<"Before sending joint angles........................................."<<std::endl;
        Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
        std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
        grasp_rob1_comm->sendDoubleMatrix(joint_config);
        std::cout<<"Message Sent: Double Matrix........................... \n";

        std::string direction_1 = grasp_rob1_comm->receiveString();
        start_rolling_gripper_action_green(200, direction_1);        
        grasp_rob1_comm->sendString("grippers_started");
        std::cout<<"Message Sent: grippers started........................... \n";


        grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "stop grippers"
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
        start_rolling_gripper_action_green(0, direction_1);
        
        grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "initialized_grasp"
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

        return true;
    }

    if(req.status.compare("start_motion") == 0)
    {   
        grasp_rob1_comm->sendString("start_execution");
        std::cout<<"Message Sent: Start Execution........................... \n";
        
        grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "start_layup"
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

        if(grasp_rob1_status.compare("start_layup") == 0)
        {
            std::cout<<"Before sending first set of joint angles........................................."<<std::endl;
            Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
            Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);

            //Sending first set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
            grasp_rob1_comm->sendDoubleMatrix(joint_config);
            std::cout<<"Message Sent: Double Matrix........................... \n";
           
            //Completed first part of execution and closing the gripper rolling
            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "angles_rcvd"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
            std::cout<<"Starting gripper rolling motion"<<std::endl;
            start_rolling_gripper_action_green(200, req.direction);
            grasp_rob1_comm->sendString("grippers_started");

            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "stop_grippers"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
            start_rolling_gripper_action_green(0, req.direction);

            //Starting next set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
            grasp_rob1_comm->sendDoubleMatrix(joint_config_2);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            grasp_rob1_comm->sendString("region:");
            std::cout<<"Message Sent: region:........................... \n";

            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

            std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        }
        return true;
    }

    else if(req.status.compare("Green_robot_action_for_region_2") == 0)
    {
        grasp_rob1_comm->sendString("next_region"); 
        std::cout<<"Message Sent: Next Region........................... \n";

        grasp_rob1_status = grasp_rob1_comm->receiveString();
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
        
        if(grasp_rob1_status.compare("starting_next_region") == 0)
        {
            std::cout<<"Before sending joint angles........................................."<<std::endl;
            Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
            Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);

            //Sending first set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
            grasp_rob1_comm->sendDoubleMatrix(joint_config);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            //Completed first part of execution and closing the gripper rolling
            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
            std::cout<<"Starting gripper rolling motion"<<std::endl;
            start_rolling_gripper_action_green(200, req.direction);
            grasp_rob1_comm->sendString("grippers_started");

            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "stop_grippers"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
            start_rolling_gripper_action_green(0, req.direction);

            //Starting next set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
            grasp_rob1_comm->sendDoubleMatrix(joint_config_2);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            grasp_rob1_comm->sendString("region:");
            std::cout<<"Message Sent: region:........................... \n";

            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

            std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        }
        return true;
    }

    else if(req.status.compare("last traj_for_green_robot") == 0)
    {
        grasp_rob1_comm->sendString("next_region");
        std::cout<<"Message Sent: Next Region........................... \n";

        grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "starting next region"
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

        std::cout<<"Before sending joint angles........................................."<<std::endl;
        Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
        Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);

        //Sending first set of joint angles
        std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
        grasp_rob1_comm->sendDoubleMatrix(joint_config);
        std::cout<<"Message Sent: Double Matrix........................... \n";

        //Completed first part of execution and closing the gripper rolling
        grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "reached_goal"
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
        std::cout<<"Starting gripper rolling motion"<<std::endl;
        start_rolling_gripper_action_green(200, req.direction);
        grasp_rob1_comm->sendString("grippers_started");


        grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "stop_grippers"
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
        start_rolling_gripper_action_green(0, req.direction);

        std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
        grasp_rob1_comm->sendDoubleMatrix(joint_config_2);
        std::cout<<"Message Sent: Double Matrix........................... \n";

        grasp_rob1_comm->sendString("last_region");
        std::cout<<"Message Sent: last_region:........................... \n";

        grasp_rob1_status = grasp_rob1_comm->receiveString();
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

        std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        return true;
    }   

    else if(req.status.compare("other_traj_for_green_rob") == 0)
    {   
        grasp_rob1_comm->sendString("next_region");
        std::cout<<"Message Sent: Next Region........................... \n";
        
        grasp_rob1_status = grasp_rob1_comm->receiveString();
        std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
        
        if(grasp_rob1_status.compare("starting_next_region") == 0)
        {
            std::cout<<"Before sending joint angles........................................."<<std::endl;
            Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
            Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);

            //Sending first set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
            grasp_rob1_comm->sendDoubleMatrix(joint_config);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            //Completed first part of execution and closing the gripper rolling
            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
            std::cout<<"Starting gripper rolling motion"<<std::endl;
            start_rolling_gripper_action_green(200, req.direction);
            grasp_rob1_comm->sendString("grippers_started");

            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "stop_grippers"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";
            start_rolling_gripper_action_green(0, req.direction);

            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
            grasp_rob1_comm->sendDoubleMatrix(joint_config_2);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            // grasp_rob1_comm->sendDoubleMatrix(xyz_cba);
            grasp_rob1_comm->sendString("region:");
            std::cout<<"Message Sent: region:........................... \n";

            grasp_rob1_status = grasp_rob1_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob1_status<<"........................... \n";

            std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        }
        return true;
    }

    else if(req.status.compare("return_to_home_pos") == 0)
    {
        grasp_rob1_comm->sendString("return_to_home_pos");
        return true;
    }
}

bool GraspingRob1Motion::getCommLayupMotionMsg(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{

    // everytime before the actual call, intervention call is received to check if there is any problem
    if(req.status.find("intervention")!=std::string::npos)
    {
        Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
        grasp_rob1_comm->sendDoubleMatrix(joint_config);
        rcvd_str = grasp_rob1_comm->receiveString();         // read conformation : "string_rcvd"     
        grasp_rob1_comm->sendString(req.status);
        rcvd_str = grasp_rob1_comm->receiveString();         // read conformation : "string_rcvd"        
        return true;
    }

    //For sheet loading operation by green robot (new)
    if(req.status.compare("sheet_loading_activity") == 0)
    {
        grasp_rob1_comm->sendString("perform_sheet_loading");
    }

    // if told to go home, then go home
    if (req.status.compare("go_home")==0)
    {
        if (toggle_grippers_during_regrasp)
        {
            gripper_status.data = "gripper_1_open";
            gripper_state_val_pub.publish(gripper_status);
        }
        grasp_rob1_comm->sendString(req.status);
        std::string gohomestr = grasp_rob1_comm->receiveString();    
        return true;
    }

    std::cout << "recahed here rob1  asasasas :: " << req.status << std::endl;
    if (req.status.compare("first_traj")==0)
    {
        std::cout << "reached here_gp1" << std::endl;  
        req.status = "next";
        gripper_status.data = "gripper_1_open";
        gripper_state_val_pub.publish(gripper_status);    
    }
    else if (req.status.compare("last_traj")!=0)
    {
        if (toggle_grippers_during_regrasp)
        {
            gripper_status.data = "gripper_1_open";
            gripper_state_val_pub.publish(gripper_status);    
        }
    }

    if (req.status.compare("last_traj_yest_grasp")==0) req.status = "last_traj";
        
    // making decision if layup already started or not
    if (!layup_started)
    {
        grasp_rob1_comm->sendString("execute_layup");
        rcvd_str = grasp_rob1_comm->receiveString();
        if (rcvd_str.compare("start_layup")==0)
        {
            layup_started = true;
        }
    } 
    // checking if the user pressed terminate layup
    if (gui_term_sig.compare("terminate_layup")==0)
    {
        goal_reached = true;
        return false;
    }
    Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
    Eigen::MatrixXd xyz_cba = ros_eigen_exch::xyz_cba_to_eigen(req.WP);
    grasp_rob1_comm->sendDoubleMatrix(joint_config);
    rcvd_str = grasp_rob1_comm->receiveString();         // read conformation to send grp index : "angles_rcvd"
    grasp_rob1_comm->sendString(req.status);
    if(grasp_rob1_comm->receiveString().compare("okay_terminating")==0)
    {
        goal_reached = true;
        layup_started = false;
    }
    return true;
}

void GraspingRob1Motion::checkIfMotionFinished(const std_msgs::String str)
{
    if (str.data.compare("motion_finished")==0)
    {
        goal_reached = true;
        gripper_status.data = "gripper_1_close";
        gripper_state_val_pub.publish(gripper_status);
    }
}

bool GraspingRob1Motion::checkMotionSynced(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{
    if (goal_reached)
    {
        res.status = "synced";
        goal_reached = false;
        return true;
    }
    else
    {
        res.status = "not_synced";
        return true;    
    }
    return true;
};

void GraspingRob1Motion::getCommState(const std_msgs::String& str)
{   //flag_to_check_comm
    if (str.data.compare("START")==0)
    {
        if(flag_to_check_comm == false){
            if(!establish_comm())
            {
                ROS_INFO("Connection could not be established for grasping robot 1 motion");    
            }
            else
            {
                flag_to_check_comm = true;
                status = true;
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

bool GraspingRob1Motion::establish_comm()
{
    return grasp_rob1_comm->establishComm();
};

void GraspingRob1Motion::close_comm()
{
    grasp_rob1_comm->sendString("terminate_connection");
    grasp_rob1_comm->closeComm();   
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Grasping_Robot_1_Motion");
    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    std::string ip_val, regrasp_toggle;
    int port_val;
    if (argc==4)
    {
        ip_val = argv[1];
        std::string s = argv[2];
        port_val = std::stoi(s);
        regrasp_toggle = argv[3];
    }

    GraspingRob1Motion GRMObj(ip_val, port_val, regrasp_toggle);
    
    while(ros::ok())
    {
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // ros::waitForShutdown();

    GRMObj.close_comm();

    return 0;
} 