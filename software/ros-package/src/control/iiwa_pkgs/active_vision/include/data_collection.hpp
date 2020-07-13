#include <TaskPlanner.hpp>
#include <iostream>
#include <active_vision/PushCommand.h>

#include <geometry_msgs/PoseStamped.h>

class CollectData{
private:
	const std::string gripperMode = "p";

	// TaskPlanner task_planner;

	std::vector<double> init_pos = {0.5,0.0,0.7,-0.38268343, 0.92387953, 0.0, 0.0};
	geometry_msgs::PoseStamped target_pose;
	geometry_msgs::PoseStamped init_pose;

public:
	TaskPlanner task_planner;
	CollectData();
	bool push(geometry_msgs::PoseStamped start_point,
				geometry_msgs::PoseStamped target_point);
};