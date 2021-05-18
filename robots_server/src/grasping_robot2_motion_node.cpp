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
#include "robots_server/grasping_robot2_motion_node.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <iostream>
#include "gen_utilities/rosmsg_eigen_exchange.hpp"
#include "gen_utilities/TrajContainerSrv.h"
#include "gen_utilities/KinectData.h"

GraspingRob2Motion::GraspingRob2Motion(std::string ip_val, int port_val, std::string regrasp_toggle)
{
    grasp_rob_mtn_pub = n_grasp_rob_mtn.advertise<sensor_msgs::JointState>("grasp_rob2_state", 1000);
    gripper_state_val_pub = n_grasp_rob_mtn.advertise<std_msgs::String>("gripper_state_val", 1000);
    comm_motion_sub = n_grasp_rob_mtn.subscribe("rob_comm_state", 1000, &GraspingRob2Motion::getCommState, this);
    
    rob_layup_motion_data_serv = n_grasp_rob_mtn.advertiseService("grasp_rob2_comm/layup_motion", &GraspingRob2Motion::getCommLayupMotionMsg, this); 
    rob_motion_data_sub_ch2 = n_grasp_rob_mtn.subscribe("gui_rob_ctrl", 1000, &GraspingRob2Motion::getGUISignalMsg, this);
    rob_layup_motion_terminator_sub = n_grasp_rob_mtn.subscribe("grasp_rob_comm/terminator", 1000, &GraspingRob2Motion::getTerminationMsg, this);
    blue_rob_grasping_mtn = n_grasp_rob_mtn.advertiseService("grasp_rob2_comm/grasping_motion", &GraspingRob2Motion::getCommGraspingMotion, this);
    kinect_rob_blue_motion_data_pub = n_grasp_rob_mtn.advertiseService("grasp_rob2_comm/kinect_motion", &GraspingRob2Motion::StrtKinectMotion, this);
    sheet_load_rob_blue_motion_data_pub = n_grasp_rob_mtn.advertiseService("grasp_rob2_comm/sheet_loading_mtn", &GraspingRob2Motion::StrtSheetLoadingMotion, this);
    gripper_rotation_pub_blue = n_grasp_rob_mtn.advertise<std_msgs::Int16>("robot1_speed_mmps",1000);

    ip = ip_val;
    port = port_val;
    grasp_rob2_comm = new robot_comm::RobotComm(ip,port,"server");
    str_val.data = "motion";
    gui_term_sig = "no_terminate";

    if (regrasp_toggle.compare("yes")==0)
    {
        toggle_grippers_during_regrasp = true;        
    }
    else
    {
        toggle_grippers_during_regrasp = false;            
    }
};

GraspingRob2Motion::~GraspingRob2Motion()
{
    std::cout<<"Destructor for blue called \n";    
}


void GraspingRob2Motion::getGUISignalMsg(const std_msgs::String& str)
{    
    gui_term_sig = str.data;
}

void GraspingRob2Motion::getTerminationMsg(const std_msgs::String str)
{
    if (str.data.compare("terminate_layup")==0)
    {
        Eigen::MatrixXd terminator_mat = Eigen::MatrixXd::Zero(1,1);
        grasp_rob2_comm->sendDoubleMatrix(terminator_mat);
        rcvd_str = grasp_rob2_comm->receiveString();  // receive the terminating confirmation from robot
        layup_started = false;
        gui_term_sig = "no_terminate";
    }
}

bool GraspingRob2Motion::StrtSheetLoadingMotion(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{
    std::cout<<"In service for executing blue robot motion for sheet loading........................... \n";
    grasp_rob2_comm->sendString("perform_sheet_loading");
    std::cout<<"Message Sent: perform_sheet_loading........................... \n";

    grasp_rob2_status = grasp_rob2_comm->receiveString(); //completed sheet loading
    std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
    
    return true;  
}

bool GraspingRob2Motion::StrtKinectMotion(gen_utilities::KinectData::Request &req,
        gen_utilities::KinectData::Response &res)
{
    if (req.status.compare("start_kinect") == 0)
    {
        grasp_rob2_comm->sendString("place_sheet");
    }
    else if(req.status.compare("moving_robots") == 0)
    {
        //grasp_rob2_comm->sendString("start_tracking");
        grasp_rob2_comm->sendDouble(req.distance);
    }
    else if(req.status.compare("stop_robots") == 0)
    {
        grasp_rob2_comm->sendString("stop_robots");
    } 
}

bool GraspingRob2Motion::start_rolling_gripper_action_blue(int vel, std::string direction)
{
    //vel = 200;
    if(direction == "negative"){
        vel = vel * (-1);
    }
    vel_blue_gripp.data = vel;
    std::cout<<"Before sending message for gripper_ctrl............................ \n";
    gripper_rotation_pub_blue.publish(vel_blue_gripp);
    std::cout<<"Message to gripper published \n";
}

bool GraspingRob2Motion::getCommGraspingMotion(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{
    std::cout<<"In service for starting blue robot motion........................... \n";
    if(req.status.compare("initial_layup_pos") == 0)
    {
        grasp_rob2_comm->sendString("initialize_grasp");
        std::cout<<"Message Sent: initialize_grasp........................... \n";

        std::cout<<"Before sending joint angles........................................."<<std::endl;
        Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
        std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
        grasp_rob2_comm->sendDoubleMatrix(joint_config);
        std::cout<<"Message Sent: Double Matrix........................... \n";

        std::string direction_1 = grasp_rob2_comm->receiveString();
        start_rolling_gripper_action_blue(200, direction_1);        
        grasp_rob2_comm->sendString("grippers_started");
        std::cout<<"Message Sent: grippers started........................... \n";


        grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "stop grasp"
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
        start_rolling_gripper_action_blue(0, direction_1);

        grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "initialized_grasp"
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

        return true;
    }
    if(req.status.compare("start_motion") == 0)
    {
        grasp_rob2_comm->sendString("start_execution");
        std::cout<<"Message Sent: Start Execution........................... \n";

        grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "start_layup"
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

        if(grasp_rob2_status.compare("start_layup") == 0)
        {
            std::cout<<"Before sending joint angles........................................."<<std::endl;
            Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
            Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);
            
            //Sending first set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
            grasp_rob2_comm->sendDoubleMatrix(joint_config);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            //Completed first part of execution and closing the gripper rolling
            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
            std::cout<<"Starting gripper rolling motion"<<std::endl;
            start_rolling_gripper_action_blue(200, req.direction);
            grasp_rob2_comm->sendString("grippers_started");

            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "stop_grippers"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
            start_rolling_gripper_action_blue(0, req.direction);

            //Starting next set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
            grasp_rob2_comm->sendDoubleMatrix(joint_config_2);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            grasp_rob2_comm->sendString("region:");
            std::cout<<"Message Sent: region:........................... \n";

            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

            std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        }
        return true;
    }

    else if(req.status.compare("Blue_robot_action_for_region_2") == 0)
    {
        grasp_rob2_comm->sendString("next_region"); //region 2 start
        std::cout<<"Message Sent: Next Region........................... \n";

        grasp_rob2_status = grasp_rob2_comm->receiveString();
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

        if(grasp_rob2_status.compare("starting_next_region") == 0)
        {
            std::cout<<"Before sending joint angles........................................."<<std::endl;
            Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
            Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);
            
            //Sending first set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
            grasp_rob2_comm->sendDoubleMatrix(joint_config);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            //Completed first part of execution and closing the gripper rolling
            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
            std::cout<<"Starting gripper rolling motion"<<std::endl;
            start_rolling_gripper_action_blue(200, req.direction);
            grasp_rob2_comm->sendString("grippers_started");

            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "stop_grippers"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
            start_rolling_gripper_action_blue(0, req.direction);

            //Starting next set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
            grasp_rob2_comm->sendDoubleMatrix(joint_config_2);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            grasp_rob2_comm->sendString("region:");
            std::cout<<"Message Sent: region:........................... \n";

            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

            std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        }
        return true;
    }

    else if(req.status.compare("last traj_for_blue_robot") == 0)
    {
        grasp_rob2_comm->sendString("next_region");
        std::cout<<"Message Sent: Next Region........................... \n";

        grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "starting next region"
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

        std::cout<<"Before sending joint angles........................................."<<std::endl;
        Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
        Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);

        //Sending first set of joint angles    
        std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
        grasp_rob2_comm->sendDoubleMatrix(joint_config);
        std::cout<<"Message Sent: Double Matrix........................... \n";

        //Completed first part of execution and closing the gripper rolling
        grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "reached_goal"
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
        std::cout<<"Starting gripper rolling motion"<<std::endl;
        start_rolling_gripper_action_blue(200, req.direction);
        grasp_rob2_comm->sendString("grippers_started");

        grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "stop_grippers"
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
        start_rolling_gripper_action_blue(0, req.direction);

        //Starting next set of joint angles
        std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
        grasp_rob2_comm->sendDoubleMatrix(joint_config_2);
        std::cout<<"Message Sent: Double Matrix........................... \n";

        grasp_rob2_comm->sendString("last_region");
        std::cout<<"Message Sent: last_region:........................... \n";

        grasp_rob2_status = grasp_rob2_comm->receiveString();
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

        std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        return true;
    }   

    else if(req.status.compare("other_traj_for_blue_rob") == 0)
    {   
        grasp_rob2_comm->sendString("next_region");
        std::cout<<"Message Sent: Next Region........................... \n";

        grasp_rob2_status = grasp_rob2_comm->receiveString();
        std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

        if(grasp_rob2_status.compare("starting_next_region") == 0)
        {
            std::cout<<"Before sending joint angles........................................."<<std::endl;
            Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
            Eigen::MatrixXd joint_config_2 = ros_eigen_exch::joint_angles_to_eigen(req.J_2);

            //Sending first set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config<<std::endl;
            grasp_rob2_comm->sendDoubleMatrix(joint_config);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            //Completed first part of execution and closing the gripper rolling
            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
            std::cout<<"Starting gripper rolling motion"<<std::endl;
            start_rolling_gripper_action_blue(200, req.direction);
            grasp_rob2_comm->sendString("grippers_started");

            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "stop_grippers"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";
            start_rolling_gripper_action_blue(0, req.direction);

            //Starting next set of joint angles
            std::cout<<"Joint angles to be sent are...............................: \n"<<joint_config_2<<std::endl;
            grasp_rob2_comm->sendDoubleMatrix(joint_config_2);
            std::cout<<"Message Sent: Double Matrix........................... \n";

            grasp_rob2_comm->sendString("region:");
            std::cout<<"Message Sent: region:........................... \n";

            grasp_rob2_status = grasp_rob2_comm->receiveString(); //getting "reached_goal"
            std::cout<<"Message Received: "<<grasp_rob2_status<<"........................... \n";

            std::cout<<"Service execution completed by robot server packages as requested by coordinator................... \n";
        }
        return true;
    }

    else if(req.status.compare("return_to_home_pos") == 0)
    {
        grasp_rob2_comm->sendString("return_to_home_pos");
        return true;
    }
}


bool GraspingRob2Motion::getCommLayupMotionMsg(gen_utilities::TrajContainerSrv::Request &req,
        gen_utilities::TrajContainerSrv::Response &res)
{   

    // everytime before the actual call, intervention call is received to check if there is any problem
    if(req.status.find("intervention")!=std::string::npos)
    {
        Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
        grasp_rob2_comm->sendDoubleMatrix(joint_config);
        rcvd_str = grasp_rob2_comm->receiveString();         // read conformation : "string_rcvd"     
        grasp_rob2_comm->sendString(req.status);
        rcvd_str = grasp_rob2_comm->receiveString();         // read conformation : "string_rcvd"        
        return true;
    }

    // if told to go home, then go home
    if (req.status.compare("go_home")==0)
    {
        if (toggle_grippers_during_regrasp)
        {
            ros::Duration(0.05).sleep();
            gripper_status.data = "gripper_2_open";
            gripper_state_val_pub.publish(gripper_status);
        }
        grasp_rob2_comm->sendString(req.status);
        std::string gohomestr =  grasp_rob2_comm->receiveString();    
        // std::cout << "grasp 2 received : " << gohomestr << std::endl;
        return true;
    }

    std::cout << "recahed here rob2  asasasas :: " << req.status << std::endl;

    if (req.status.compare("first_traj")==0)
    {
        std::cout << "reached here_gp2" << std::endl;  
        req.status = "next";
        ros::Duration(0.05).sleep();
        gripper_status.data = "gripper_2_open";
        gripper_state_val_pub.publish(gripper_status);
        ros::Duration(1).sleep();
    }
    else if (req.status.compare("last_traj")!=0)
    {
        if (toggle_grippers_during_regrasp)
        {
            ros::Duration(0.05).sleep();
            gripper_status.data = "gripper_2_open";
            gripper_state_val_pub.publish(gripper_status);
            ros::Duration(1).sleep();
        }
    }

    if (req.status.compare("last_traj_yest_grasp")==0) req.status = "last_traj";
    
     
    // making decision if layup already started or not
    if (!layup_started)
    {
        grasp_rob2_comm->sendString("execute_layup");
        rcvd_str = grasp_rob2_comm->receiveString();
        if (rcvd_str.compare("start_layup")==0)
        {
            layup_started = true;
        }
    } 
    // checking if the user pressed terminate layup
    if (gui_term_sig.compare("terminate_layup")==0)
    {
        return false;
    }
    Eigen::MatrixXd joint_config = ros_eigen_exch::joint_angles_to_eigen(req.J);
    Eigen::MatrixXd xyz_cba = ros_eigen_exch::xyz_cba_to_eigen(req.WP);
    grasp_rob2_comm->sendDoubleMatrix(joint_config);
    rcvd_str = grasp_rob2_comm->receiveString();         

    gripper_status.data = "gripper_2_close";
    gripper_state_val_pub.publish(gripper_status); 

    grasp_rob2_comm->sendString(req.status);
    if(grasp_rob2_comm->receiveString().compare("okay_terminating")==0)
    {
        layup_started = false;
    }
    return true;
}

void GraspingRob2Motion::getCommState(const std_msgs::String& str)
{
    std::cout<<"Values of flag is: Grasp Rob2Inside func "<<flag_to_check_comm<<std::endl;
    if (str.data.compare("START")==0)
    {
        std::cout<<"Values of flag is: Grasp Rob2 Inside stop "<<flag_to_check_comm<<std::endl;
        if(flag_to_check_comm == false){
                establish_comm();
                //status = true;
                flag_to_check_comm = true;
                std::cout<<"Values of flag is: Grasp Rob2 Inside start "<<flag_to_check_comm<<std::endl;
            }
    }
    if (str.data.compare("STOP")==0)
    {
        close_comm();
        status = false;
        flag_to_check_comm = false;
    }
    std::cout<<"Values of flag is: Grasp Rob2 after func "<<flag_to_check_comm<<std::endl;
};

bool GraspingRob2Motion::establish_comm()
{
    return grasp_rob2_comm->establishComm();
};

void GraspingRob2Motion::close_comm()
{
    grasp_rob2_comm->sendString("terminate_connection");
    grasp_rob2_comm->closeComm();   
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Grasping_Robot_2_Motion");
    // ros::AsyncSpinner spinner(2);
    // spinner.start();


    std::string ip_val,regrasp_toggle;
    int port_val;
    if (argc==4)
    {
        ip_val = argv[1];
        std::string s = argv[2];
        port_val = std::stoi(s);
        regrasp_toggle = argv[3];
    }

    GraspingRob2Motion GRMObj(ip_val, port_val, regrasp_toggle);
    
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