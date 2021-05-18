#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <fstream>
#include <stdio.h>

void savetocsv( sensor_msgs::JointState states){
    float a [6];
    for(int i=0;i<6;i++){
        a[i] = states.position[i];

    }

    FILE *myFile = NULL;
    
    // ROS_INFO("I heard: [%s]", states.position->data.c_str());
    // std::ofstream myFile("yas_joint_states.csv");

    myFile = fopen("abb_joint_states.csv","a");
    fprintf(myFile,"%f,%f,%f,%f,%f,%f\n",a[0],a[1],a[2],a[3],a[4],a[5]);
    // myFile<<a[0]<<","<<a[1]<<","<<a[2]<<","<<a[3]<<","<<a[4]<<","<<a[5]<<","<<a[6]<<"\n";
    // myFile.close();
    fclose(myFile);
}

int main(int argc, char **argv)
{
	ROS_INFO("Starting Node abb_joint_states_to_csv");
	ros::init(argc, argv, "abb_joint_states_to_csv");
	ros::NodeHandle n;
    ros::Rate rate(1);



    ros::Subscriber sub = n.subscribe("abb_rob/joint_states", 1, savetocsv);

    ros::spin();
    return 0;
}