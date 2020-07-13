#include "ros/ros.h"
#include <ros/package.h>
#include <vector>
#include <task_planning/PlacePlanMultiAction.h> 
#include <placement_module.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<task_planning::PlacePlanMultiAction> PlacePlanMultiClient;
std::vector<geometry_msgs::Pose> pre_push_pose_;
geometry_msgs::Pose post_push_pose_;

bool is_placeplan_ready_ = false;
void placeplan_doneCb(const actionlib::SimpleClientGoalState& state,
            const task_planning::PlacePlanMultiResultConstPtr& result)
{
  ROS_WARN("Finished the placeplan action");
  //ROS_INFO("Answer: %i", result->sequence.back());
  pre_push_pose_ = result->pre_push_pose; 
  post_push_pose_ = result->post_push_pose; 
  is_placeplan_ready_ = true;

}

void placeplan_activeCb()
{
  ROS_INFO("armmove Goal just went active");
}

// Called every time feedback is received for the goal
void placeplan_feedbackCb(const task_planning::PlacePlanMultiFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}


int main(int argc, char** argv){
	ros::init(argc, argv, "placeplan_action_client");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;
	PlacePlanMultiClient ac("/placeplanmulti_action", true);
	ac.waitForServer();
	task_planning::PlacePlanMultiGoal placeplan_goal;
	geometry_msgs::Pose target_obj_pose;
	 double x_calibration_offset = -0.008;
	 double y_calibration_offset = 0.009;
	// double x_calibration_offset = -0.0;
	// double y_calibration_offset = 0.0;
	 // 0.396625,-0.209541
	placement_module placement_mod;
	int obj_num_per_layer = placement_mod.setup("dove");
	std::string input;
	int grid_id;
	double offset;

	while(1){ 

		ROS_WARN_STREAM("Starting loop");
		std::cout<<"input z-offset";
		std::cin>>offset;

		std::cout<<"input grid_id:";
		std::cin>>grid_id;
		target_obj_pose = placement_mod.get_grid_pose("bin_1", grid_id);
		// target_obj_pose.position.x = 0.396625 + x_calibration_offset;
		// target_obj_pose.position.y = -0.209541 + y_calibration_offset;
		// target_obj_pose.position.z = -0.18;
		// target_obj_pose.orientation.x = 0;
		// target_obj_pose.orientation.y = 0;
		// target_obj_pose.orientation.z = 0.5;
		// target_obj_pose.orientation.w = 0.8660254;

	    placeplan_goal.target_object_pose = target_obj_pose;
	    placeplan_goal.target_object_name = "dove";
	    ac.sendGoal(placeplan_goal, &placeplan_doneCb, &placeplan_activeCb, &placeplan_feedbackCb);
	    ros::spinOnce();
	    while(!is_placeplan_ready_);
	    std::cout<<"Enter something to continue...";
	    std::cin>>input;
	    ROS_WARN_STREAM("Restarting loop...");
	    // std::std::flush;
	    target_obj_pose.position.z += offset;
	    placeplan_goal.target_object_pose = target_obj_pose;
	    ac.sendGoal(placeplan_goal, &placeplan_doneCb, &placeplan_activeCb, &placeplan_feedbackCb);
	    ros::spinOnce();
	    while(!is_placeplan_ready_);
	}
    while(!is_placeplan_ready_);
    ROS_WARN("placeplan action returned!!!!!!!!!!!!!");

    return 1;
}