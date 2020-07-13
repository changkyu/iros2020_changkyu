/*
File: Painting.cpp

Authors: Aravind Sivaramakrishnan

Description:

Comments/TODO:
- Should call services for prediction.
- object_poses should be an array of geometry_msgs/PoseStamped. Or shouldn't it?
- I think we should add the canvas as an obstacle for motion planning.
*/
#include <Painting.hpp>

Painting::Painting()
{
	// std::default_random_engine generator;
	// std::uniform_real_distribution<double> distribution(0.0,1.0);

	// ROS_INFO_STREAM(distribution(generator)); 

	obj_srv.request.normal.x = 0;
	obj_srv.request.normal.y = 0;
	obj_srv.request.normal.z = -1; 

	if (obj_client.call(obj_srv))
	{
		ROS_INFO_STREAM("Perception service successfully called...");
		update_object_poses();
	}
	else
	{
		ROS_ERROR("Failed to call segmentation service...");
	}

	object_poses["bucket"] = {0.6, 0.25,0.025,0.27059805,  0.65328148,  0.65328148, -0.27059805};
	// object_poses["canvas"] = {0.66306575,  -0.25292536, 0.15738983,
		// -0.27059805,  0.65328148,  0.65328148,  0.27059805};

	object_poses["aim"] =  {0.6,0.0,0.65,-0.27059805,  0.65328148,  0.65328148,  0.27059805};
}

bool Painting::run_experiment()
{
	while(ros::ok())
	{
		ROS_INFO_STREAM("Current state at Painting: " << get_state(current_state));
		if (current_state == START)
		{
			execution_result = task_planner.executeTask("HOME");
			current_state = execution_result ? PICK_BRUSH : FINISH; 
		}
		else if (current_state == PICK_BRUSH)
		{
			grasp_pose.pose.position.x = object_poses["brush"].at(0);
			grasp_pose.pose.position.y = object_poses["brush"].at(1);
			grasp_pose.pose.position.z = object_poses["brush"].at(2);
			grasp_pose.pose.orientation.x = object_poses["brush"].at(3);
			grasp_pose.pose.orientation.y = object_poses["brush"].at(4);
			grasp_pose.pose.orientation.z = object_poses["brush"].at(5);
			grasp_pose.pose.orientation.w = object_poses["brush"].at(6);

			execution_result = task_planner.executeTask("PICK",grasp_pose,10.0);
			current_state = execution_result ? PRE_DIP : FINISH;
		}
		else if (current_state == PRE_DIP)
		{
			offset = {0.0,0.0,0.2};
			set_target("bucket");

			execution_result = task_planner.executeTask("MOVE",target_pose);
			current_state = execution_result ? DIP : FINISH;
		}
		else if (current_state == DIP)
		{
			offset = {0.0,0.0,0.0};
			set_target("bucket");
			
			execution_result = task_planner.executeTask("STEER",target_pose);
			if (!execution_result)
			{
				ROS_INFO_STREAM("Steering failed!");
				execution_result = task_planner.executeTask("MOVE",target_pose);
			}
			current_state = execution_result ? POST_DIP : FINISH;
		}
		else if (current_state == POST_DIP)
		{
			offset = {0.0,0.0,0.25};
			set_target("bucket");
	
			execution_result = task_planner.executeTask("MOVE",target_pose);
			current_state = execution_result ? AIM : FINISH;
		}
		else if (current_state == AIM)
		{
			offset = {0.0,0.0,0.0};
			set_target("aim");

			execution_result = task_planner.executeTask("MOVE",target_pose);
			current_state = execution_result ? TOUCH_CANVAS : FINISH;
		}
		else if (current_state == TOUCH_CANVAS)
		{
			offset = {0.0,0.0,0.0};
			set_target("canvas");

			execution_result = task_planner.executeTask("MOVE",target_pose);
			current_state = execution_result ? PAINT : FINISH;
		}
		else if (current_state == PAINT)
		{
			offset = {0.0,0.3,0.0};
			set_target("canvas");

			execution_result = task_planner.executeTask("STEER",target_pose);
			current_state = execution_result ? FINISH: FINISH;

		}
		else if (current_state == FINISH)
		{
			return true;
		}
		loop_rate_->sleep();
	}
}

std::string Painting::get_state(State current_state)
{
	switch(current_state)
	{
		case START: return "START";
		case PICK_BRUSH: return "PICK BRUSH";
		case PRE_DIP: return "PRE_DIP";
		case DIP: return "DIP";
		case POST_DIP: return "POST_DIP";
		case TOUCH_CANVAS: return "TOUCH_CANVAS";
		case PAINT: return "PAINT";
		case FINISH: return "FINISH";
		case AIM: return "AIM";
		default: return "Bad state!";
	}
}

void Painting::set_target(std::string object)
{
	target_pose.pose.position.x = object_poses[object].at(0) + offset.at(0);
	target_pose.pose.position.y = object_poses[object].at(1) + offset.at(1);
	target_pose.pose.position.z = object_poses[object].at(2) + offset.at(2);
	target_pose.pose.orientation.x = object_poses[object].at(3);
	target_pose.pose.orientation.y = object_poses[object].at(4);
	target_pose.pose.orientation.z = object_poses[object].at(5);
	target_pose.pose.orientation.w = object_poses[object].at(6);
}

void Painting::update_object_poses()
{
	for (std::vector<task_planning::ObjectPose>::const_iterator it = obj_srv.response.object_messages.begin(); it != obj_srv.response.object_messages.end(); ++it)
	{
		ROS_INFO_STREAM("Updating the pose for " << it->label);	
		object_poses[it->label].push_back(it->pose.position.x);
		object_poses[it->label].push_back(it->pose.position.y);
		object_poses[it->label].push_back(it->pose.position.z);
		object_poses[it->label].push_back(it->pose.orientation.x);
		object_poses[it->label].push_back(it->pose.orientation.y);
		object_poses[it->label].push_back(it->pose.orientation.z);
		object_poses[it->label].push_back(it->pose.orientation.w);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "painting_experiment");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	Painting painting;

	bool success = painting.run_experiment();

	return 0;
}
