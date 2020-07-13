/*
File: Stacking.cpp

Authors: Aravind Sivaramakrishnan

Description: 

Comments/TODO:
- Should call services for prediction.
- object_poses should be an array of geometry_msgs/PoseStamped. Or shouldn't it?
*/
#include <Stacking.hpp>

Stacking::Stacking()
{
	prediction_result = "N";
	object_poses["place"] = {0.6,0.35,0.15,-0.38268343, 0.92387953, 0.0, 0.0};
}

bool Stacking::run_experiment()
{
	while (ros::ok())
	{
		ROS_INFO_STREAM("Current state at Stacking: " << get_state(current_state));

		if (current_state == START)
		{
			execution_result = task_planner.executeTask("HOME");
			current_state = execution_result ? SENSE : FINISH;
		}
		else if (current_state == SENSE)
		{
			obj_srv.request.normal.x = 0;
			obj_srv.request.normal.y = 0;
			obj_srv.request.normal.z = -1; 

			if (obj_client.call(obj_srv))
			{
				ROS_INFO_STREAM("Perception service successfully called...");
				update_object_poses_and_sizes();
				current_state = WAIT_FOR_PREDICTION;
			}
			else
			{
				ROS_ERROR("Failed to call segmentation service...");
				current_state = SENSE;
			}
		}
		else if (current_state == WAIT_FOR_PREDICTION)
		{
			if (prediction_result == "N")
				prediction_result = "B";
			else if (prediction_result == "B")
				prediction_result = "C";
			else
			{
				current_state = FINISH;
				continue;
			}
			ROS_INFO_STREAM("Prediction is " << prediction_result);
			set_target(prediction_result);
			current_state = PICK;
		}
		else if (current_state == PICK)
		{
			execution_result = task_planner.executeTask("PICK",target_pose,object_sizes[prediction_result].at(0));
			current_state = execution_result ? PLACE : FINISH;
		}
		else if (current_state == PLACE)
		{
			set_target("place");
			execution_result = task_planner.executeTask("PLACE",target_pose);
			object_poses["place"].at(2) += 0.05;			
			current_state = execution_result ? WAIT_FOR_PREDICTION : FINISH;
		}
		else if (current_state == FINISH)
		{
			return true;
		}
		loop_rate_->sleep();
	}
}

std::string Stacking::get_state(State current_state)
{
	switch(current_state)
	{
		case START: return "START";
		case SENSE: return "SENSE";
		case WAIT_FOR_PREDICTION: return "WAIT_FOR_PREDICTION";
		case PICK: return "PICK";
		case PLACE: return "PLACE";
		case FINISH: return "FINISH";
		default: return "Bad state!";
	}
}

void Stacking::update_object_poses_and_sizes()
{
	for (std::vector<task_planning::ObjectPose>::const_iterator it = obj_srv.response.object_messages.begin(); it != obj_srv.response.object_messages.end(); ++it)
	{
		ROS_INFO_STREAM("Updating the pose for object " << it->label);
		object_poses[it->label].push_back(it->pose.position.x);
		object_poses[it->label].push_back(it->pose.position.y);
		object_poses[it->label].push_back(it->pose.position.z);
		object_poses[it->label].push_back(it->pose.orientation.x);
		object_poses[it->label].push_back(it->pose.orientation.y);
		object_poses[it->label].push_back(it->pose.orientation.z);
		object_poses[it->label].push_back(it->pose.orientation.w);

		for (std::vector<std_msgs::Float32>::const_iterator it2 = it->sizes.begin(); it2 != it->sizes.end(); ++it2)
			object_sizes[it->label].push_back(it2->data);
	}
}

void Stacking::set_target(std::string object)
{
	target_pose.pose.position.x = object_poses[object].at(0);
	target_pose.pose.position.y = object_poses[object].at(1);
	target_pose.pose.position.z = object_poses[object].at(2);
	target_pose.pose.orientation.x = object_poses[object].at(3);
	target_pose.pose.orientation.y = object_poses[object].at(4);
	target_pose.pose.orientation.z = object_poses[object].at(5);
	target_pose.pose.orientation.w = object_poses[object].at(6);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "stacking_experiment");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	Stacking stacking;

	bool success = stacking.run_experiment();

	return 0;
}