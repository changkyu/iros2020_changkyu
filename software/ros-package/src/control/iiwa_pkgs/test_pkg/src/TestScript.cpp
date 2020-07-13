/*
File: TestScript.cpp

Authors: Aravind Sivaramakrishnan

Description: Use this script to test out a particular functionality of the robot.

Comments/TODO:
- Two instances of iiwa_ros, this is not good, but let it go since it's only the test script.
*/
#include <string.h>
#include <math.h>
#include <utils.hpp>
#include <TaskPlanner.hpp>
#include <iiwa_msgs/JointPosition.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char**argv)
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh("~");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	TaskPlanner task_planner;

	iiwa_ros::iiwaRos iiwa_copy;
	iiwa_copy.init();

	std::string mode;
	nh.param<std::string>("mode",mode,"pose");

	double dist;
	nh.param("dist",dist,0.2);
	double num_midpts = 5;

	geometry_msgs::PoseStamped command_cartesian_pose;
	geometry_msgs::PoseStamped current_pose;

	std::vector<double> quat = {-0.38268343, 0.92387953, 0.0, 0.0};
	// std::vector<double> quat = {0.27059805,  0.65328148,  0.65328148, -0.27059805};
	std::vector<double> pos = {0.65,-0.45,0.30};

	std::vector<double> place_pos = {0.6,-0.55,0.25,-0.38268343, 0.92387953, 0.0, 0.0};

	while (ros::ok())
	{
		if (iiwa_copy.getRobotIsConnected())
		{
			ROS_INFO("Robot is connected...");
			if (mode == "pose")
			{
				ROS_INFO("Steering to desired pose...");
				while (!iiwa_copy.getCartesianPose(current_pose)) {}
				command_cartesian_pose = current_pose;
				command_cartesian_pose.pose.position.z += dist;
				iiwa_copy.setCartesianPose(command_cartesian_pose);
				// ROS_INFO_STREAM("Success? " << task_planner.executeTask("STEER",command_cartesian_pose));
			}
			else if (mode == "home")
			{
				ROS_INFO("Moving to home position...");
				ROS_INFO_STREAM("Success?" << task_planner.executeTask("HOME"));
			}
			else if (mode == "moveit")
			{
				ROS_INFO("Testing motion planning...");
				while (!iiwa_copy.getCartesianPose(command_cartesian_pose)) {}
				command_cartesian_pose.pose.position.x = pos.at(0);
				command_cartesian_pose.pose.position.y = pos.at(1);
				command_cartesian_pose.pose.position.z = pos.at(2);
				command_cartesian_pose.pose.orientation.x = quat.at(0);
				command_cartesian_pose.pose.orientation.y = quat.at(1);
				command_cartesian_pose.pose.orientation.z = quat.at(2);
				command_cartesian_pose.pose.orientation.w = quat.at(3);
				ROS_INFO_STREAM("Success? " << task_planner.executeTask("MOVE",command_cartesian_pose));
			}
			else if (mode == "grasp")
			{
				ROS_INFO("Testing grasping...");
				command_cartesian_pose.pose.position.x = pos.at(0);
				command_cartesian_pose.pose.position.y = pos.at(1);
				command_cartesian_pose.pose.position.z = pos.at(2);
				command_cartesian_pose.pose.orientation.x = quat.at(0);
				command_cartesian_pose.pose.orientation.y = quat.at(1);
				command_cartesian_pose.pose.orientation.z = quat.at(2);
				command_cartesian_pose.pose.orientation.w = quat.at(3);
				ROS_INFO_STREAM("Success?" << task_planner.executeTask("PICK",command_cartesian_pose,0.55));
			}
			else if (mode == "pickplace")
			{
				ROS_INFO("Testing pick and place...");

				geometry_msgs::PoseStamped place_pose;
				place_pose.pose.position.x = place_pos.at(0);
				place_pose.pose.position.y = place_pos.at(1);
				place_pose.pose.position.z = place_pos.at(2);
				place_pose.pose.orientation.x = place_pos.at(3);
				place_pose.pose.orientation.y = place_pos.at(4);
				place_pose.pose.orientation.z = place_pos.at(5);
				place_pose.pose.orientation.w = place_pos.at(6);

				command_cartesian_pose.pose.position.x = pos.at(0);
				command_cartesian_pose.pose.position.y = pos.at(1);
				command_cartesian_pose.pose.position.z = pos.at(2);
				command_cartesian_pose.pose.orientation.x = quat.at(0);
				command_cartesian_pose.pose.orientation.y = quat.at(1);
				command_cartesian_pose.pose.orientation.z = quat.at(2);
				command_cartesian_pose.pose.orientation.w = quat.at(3);

				//ROS_INFO_STREAM("Success?" << task_planner.executeTask("PICK_AND_PLACE",command_cartesian_pose,place_pose,0.55));
			}
			ROS_INFO("Task successfully executed!");
			if (mode != "test")
				ros::shutdown();
		}
	}
}