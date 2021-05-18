#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include <actionlib/client/terminal_state.h>

int main(int argc, char **argv)
{
	ROS_INFO("Starting Node robot_trajectory_node");
	ros::init(argc, argv, "robot_trajectory_node");
	ros::NodeHandle n;

	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/joint_trajectory_action");
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");

	control_msgs::FollowJointTrajectoryGoal curr_goal;

	trajectory_msgs::JointTrajectory new_joint_trajectory;

	new_joint_trajectory.header.stamp = ros::Time::now();
	new_joint_trajectory.header.frame_id = "temp_pose";
	new_joint_trajectory.joint_names.resize(7);
	new_joint_trajectory.points.resize(2);

	new_joint_trajectory.joint_names[0] = "joint_s";
	new_joint_trajectory.joint_names[1] = "joint_l";
	new_joint_trajectory.joint_names[2] = "joint_e";
	new_joint_trajectory.joint_names[3] = "joint_u";
	new_joint_trajectory.joint_names[4] = "joint_r";
	new_joint_trajectory.joint_names[5] = "joint_b";
	new_joint_trajectory.joint_names[6] = "joint_t";

	trajectory_msgs::JointTrajectoryPoint next_joint_values;

	// first waypoint
	next_joint_values.positions.resize(7);
	next_joint_values.positions[0] = 0.5257123112678528;
	next_joint_values.positions[1] = 0.12934868037700653;
	next_joint_values.positions[2] = 0.01880265586078167;
	next_joint_values.positions[3] = -0.5712483525276184;
	next_joint_values.positions[4] = 0.04472844675183296;
	next_joint_values.positions[5] = 0.9336728835105896;
	next_joint_values.positions[6] = -1.9051140546798706;

	ROS_INFO("Positions0.");

	next_joint_values.velocities.resize(7);
	next_joint_values.velocities[0] = 0.0;
	next_joint_values.velocities[1] = 0.0;
	next_joint_values.velocities[2] = 0.0;
	next_joint_values.velocities[3] = 0.0;
	next_joint_values.velocities[4] = 0.0;
	next_joint_values.velocities[5] = 0.0;
	next_joint_values.velocities[6] = 0.0;

	ROS_INFO("Positions1.");


	new_joint_trajectory.points[0] = next_joint_values;
	

	// second waypoint
next_joint_values.positions.resize(7);
	next_joint_values.positions[0] = 0.5257123112678528;
	next_joint_values.positions[1] = 0.12934868037700653;
	next_joint_values.positions[2] = 0.01880265586078167;
	next_joint_values.positions[3] = -0.5712483525276184;
	next_joint_values.positions[4] = 0.04472844675183296;
	next_joint_values.positions[5] = 0.8336728835105896;
	next_joint_values.positions[6] = -1.9051140546798706;

	next_joint_values.velocities.resize(7);
	next_joint_values.velocities[0] = 0.0;
	next_joint_values.velocities[1] = 0.0;
	next_joint_values.velocities[2] = 0.0;
	next_joint_values.velocities[3] = 0.0;
	next_joint_values.velocities[4] = 0.0;
	next_joint_values.velocities[5] = 0.0;
	next_joint_values.velocities[6] = 0.0;

	next_joint_values.time_from_start = ros::Duration(0.01);

	new_joint_trajectory.points[1] = next_joint_values;

	curr_goal.trajectory = new_joint_trajectory;

	ac.sendGoal(curr_goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}

	return 0;
}