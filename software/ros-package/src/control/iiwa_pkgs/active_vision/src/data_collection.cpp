/*
File: data_collection.cpp

Authors: Aravind Sivaramakrishnan

Description:

Comments/TODO:
- TaskPlanner is not private!!!
*/

#include <data_collection.hpp>

CollectData::CollectData():task_planner(gripperMode)
{
	task_planner.executeTask("HOME");

	init_pose.pose.position.x = init_pos.at(0);
	init_pose.pose.position.y = init_pos.at(1);
	init_pose.pose.position.z = init_pos.at(2);
	init_pose.pose.orientation.x = init_pos.at(3);
	init_pose.pose.orientation.y = init_pos.at(4);
	init_pose.pose.orientation.z = init_pos.at(5);
	init_pose.pose.orientation.w = init_pos.at(6);

	task_planner.executeTask("MOVE",init_pose);
	ROS_INFO_STREAM("Press [Enter] when you're good to go.");
	getchar();

	task_planner.executeTask("CLOSE_FINGERS");
	ROS_INFO_STREAM("Press [Enter] when you're good to go.");
	getchar();
}

bool CollectData::push(geometry_msgs::PoseStamped start_point,
				geometry_msgs::PoseStamped target_point)
{
	ROS_INFO_STREAM("Received a new service call!");

	bool success;

	success = task_planner.executeTask("MOVE",start_point);

	if (success) task_planner.executeTask("STEER",target_point);

	task_planner.executeTask("MOVE",init_pose);
}

