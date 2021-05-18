#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <fstream>
#include <stdio.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

void tcptocsv(double x, double y, double z, double rx, double ry, double rz)
{

    FILE *myFile2 = NULL;

    // ROS_INFO("I heard: [%s]", states.position->data.c_str());
    // std::ofstream myFile("yas_joint_states.csv");

    myFile2 = fopen("abb_TCP.csv", "a");
    fprintf(myFile2, "%g,%g,%g,%g,%g,%g\n", x, y, z,rx,ry,rz);

    fclose(myFile2);
}


int main(int argc, char **argv)
{
    ROS_INFO("Starting Node abb_tcp_to_csv");
    ros::init(argc, argv, "abb_tcp_to_csv");
    ros::NodeHandle n;
    
    while (ros::ok())
    {
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        geometry_msgs::TransformStamped trans;

        tfBuffer.canTransform("world", "abb_rob/TCP", ros::Time::now(), ros::Duration(1.0));

        trans = tfBuffer.lookupTransform("world", "abb_rob/TCP", ros::Time(0));

        double x, y, z, rx, ry, rz;
        x = trans.transform.translation.x;
        y = trans.transform.translation.y;
        z = trans.transform.translation.z;

        rx = trans.transform.rotation.x;
        ry = trans.transform.rotation.y;
        rz = trans.transform.rotation.z;

        tcptocsv(x, y, z, rx, ry, rz);
    }

    ros::spin();
    return 0;
}