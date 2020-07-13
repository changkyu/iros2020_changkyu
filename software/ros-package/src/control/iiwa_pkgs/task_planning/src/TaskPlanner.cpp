/*
File: TaskPlanner.cpp

Authors: Aravind Sivaramakrishnan

Description: The TaskPlanner brings motion planning and grasping together.

Comments/TODO:
- Some shady stuff happening with the overloaded constructor. For ex, MoveIt 
node's constructor is getting initialized all over again.
- Write the implementation of addCollisionObject.
*/
#include <TaskPlanner.hpp>
#include "grasping/GraspPlanning.h"  // service file
#include "grasping/GraspSuccess.h"  // service file
#include "rl_msgs/organizing_pusher_jdxdemo_srv.h"
#include <task_planning/PlacePlanAction.h> 
#include <actionlib/client/simple_action_client.h>
#include "task_planning/PlacePlan.h"
#include "task_planning/PlacePlanMultiAction.h"

#define  X_CALIBRATION_OFFSET  0
#define  Y_CALIBRATION_OFFSET  0
#define SENSING_OFFSET_TO_CENTER 0.0
#define SENSING_Z_ADJUSTMENT 0.05


geometry_msgs::Pose pre_push_pose_;
geometry_msgs::Pose post_push_pose_;
std::vector<geometry_msgs::Pose> pre_push_pose_list_;
typedef actionlib::SimpleActionClient<task_planning::PlacePlanAction> PlacePlanClient;
typedef actionlib::SimpleActionClient<task_planning::PlacePlanMultiAction> PlacePlanMultiClient;
//
#define GRASP_DETECTION_THRESHOLD 100
// #define SENSING_SLEEP 0.5
#define SENSING_SLEEP 1.0
bool is_placeplan_ready_ = false;
void placeplan_doneCb(const actionlib::SimpleClientGoalState& state,
            const task_planning::PlacePlanResultConstPtr& result)
{
  ROS_WARN("Finished the placeplan action");
  //ROS_INFO("Answer: %i", result->sequence.back());
  pre_push_pose_ = result->pre_push_pose; 
  post_push_pose_ = result->post_push_pose; 
  pre_push_pose_.position.x -= X_CALIBRATION_OFFSET;
  pre_push_pose_.position.y -= Y_CALIBRATION_OFFSET;
  post_push_pose_.position.x -= X_CALIBRATION_OFFSET;
  post_push_pose_.position.y -= Y_CALIBRATION_OFFSET;
  is_placeplan_ready_ = true;
}

void placeplan_activeCb()
{
  ROS_INFO("armmove Goal just went active");
}

// Called every time feedback is received for the goal
void placeplan_feedbackCb(const task_planning::PlacePlanFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}

bool is_placeplanmulti_ready_ = false;
void placeplanmulti_doneCb(const actionlib::SimpleClientGoalState& state,
            const task_planning::PlacePlanMultiResultConstPtr& result)
{
  ROS_WARN("Finished the placeplan action");
  //ROS_INFO("Answer: %i", result->sequence.back());
  pre_push_pose_list_ = result->pre_push_pose; 
  post_push_pose_ = result->post_push_pose; 
  is_placeplanmulti_ready_ = true;
}

void placeplanmulti_activeCb()
{
  ROS_INFO("armmove Goal just went active");
}

// Called every time feedback is received for the goal
void placeplanmulti_feedbackCb(const task_planning::PlacePlanMultiFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}


Eigen::Matrix3f get_rotation_matrix_from_orientation(geometry_msgs::Quaternion ros_quat)
{
	Eigen::Quaternionf eigen_quat;
	eigen_quat.x() = ros_quat.x;
	eigen_quat.y() = ros_quat.y;
	eigen_quat.z() = ros_quat.z;
	eigen_quat.w() = ros_quat.w;
	Eigen::Matrix3f eigen_mat = eigen_quat.toRotationMatrix();
	return eigen_mat;
}


geometry_msgs::Quaternion get_quaternion_from_eigen_quaternion(Eigen::Quaternionf eigen_quat)
{
	geometry_msgs::Quaternion ros_quat;
	ros_quat.x = eigen_quat.x();
	ros_quat.y = eigen_quat.y();
	ros_quat.z = eigen_quat.z();
	ros_quat.w = eigen_quat.w();
	return ros_quat;
}

double in_degrees(double radians)
{
	return radians*180/3.1418;
}


TaskPlanner::TaskPlanner()
{
	ROS_WARN("before init my_iiwa");
	// Initialize the iiwa_ros class.
	my_iiwa.init();

	ROS_INFO("Initialized iiwa!");
	
	// Keep checking if the gripper is active or not.
	#ifndef USE_SIMULATION
	#ifdef REFLEX
	// #### ROBOTIQ GRIPPER
	while (!isGripperActive)
	{
		gripperStatusMsg = ros::topic::waitForMessage<robotiq_s_model_control::SModel_robot_input>
							("SModelRobotInput",node_handle,ros::Duration(1.0));
		getGripperStatus();	
		ROS_INFO_STREAM("Gripper status is " << (isGripperActive ? "active":"not active"));					
	}
	#endif
	#endif
	// Initialize the gripper command publisher.
	// gripper_command_publisher = node_handle.advertise<std_msgs::String>("GripperCommand",1);
	transfer_pose_1.pose.position.x = 0.39;
	transfer_pose_1.pose.position.y = -0.34;
	transfer_pose_1.pose.position.z = 0.55;
	transfer_pose_1.pose.orientation.x = -0.38268343;
	transfer_pose_1.pose.orientation.y = 0.92387953;
	transfer_pose_1.pose.orientation.z = 0.0;
	transfer_pose_1.pose.orientation.w = 0.0;

	transfer_pose_2.pose.position.x = 0.39;
	//TESTSTETSTES
	transfer_pose_2.pose.position.x = 0.35;
	transfer_pose_2.pose.position.y = 0.34;
	transfer_pose_2.pose.position.z = 0.55;
	transfer_pose_2.pose.orientation.x = -0.38268343;
	transfer_pose_2.pose.orientation.y = 0.92387953;
	transfer_pose_2.pose.orientation.z = 0.0;
	transfer_pose_2.pose.orientation.w = 0.0;

	transfer_joint_1 = {-0.511658,0.485311,-0.313416,-1.65187,0.167873,1.03135,-1.66345};
	transfer_joint_2 = {0.296138,0.496857,0.561381,-1.69344,-0.29896,1.03727,0.17045};
	//-23.49 , 27.175 , -25.58 , 
	bin_2_back_joint = {-0.41433,0.474332,-0.446511,-1.62738,0.225128,1.09109,-1.70775};
	bin_2_go_joint = {0.726251,0.32149,-0.0139663,-1.52822,0.00992217,1.28858,-0.105161};
	bin_1_go_joint = {-0.406311,0.380705,-0.458801,-1.48475,0.171038,1.31584,-1.66451};
	bin_1_back_joint = {0.869973,0.430851,-0.0142234,-1.59738,0.0265756,1.16728,-0.139013};



	test_pose_1.pose.position.x = 0.39;
	test_pose_1.pose.position.y = -0.34;
	test_pose_1.pose.position.z = 0.08;
	test_pose_1.pose.orientation.x = -0.38268343;
	test_pose_1.pose.orientation.y = 0.92387953;
	test_pose_1.pose.orientation.z = 0.0;
	test_pose_1.pose.orientation.w = 0.0;

	test_pose_2.pose.position.x = 0.39;
	test_pose_2.pose.position.y = 0.34;
	test_pose_2.pose.position.z = 0.08;
	test_pose_2.pose.orientation.x = -0.38268343;
	test_pose_2.pose.orientation.y = 0.92387953;
	test_pose_2.pose.orientation.z = 0.0;
	test_pose_2.pose.orientation.w = 0.0;

	regrasp_failure = 0;

}

TaskPlanner::TaskPlanner(std::string gripperMode)
{
	// Initialize the iiwa_ros class.
	my_iiwa.init();

	ROS_INFO("Initialized iiwa!");

	// Keep checking if the gripper is active or not.
	#ifndef USE_SIMULATION
	while (!isGripperActive)
	{
		gripperStatusMsg = ros::topic::waitForMessage<robotiq_s_model_control::SModel_robot_input>
							("SModelRobotInput",node_handle,ros::Duration(1.0));
		getGripperStatus();	
		ROS_INFO_STREAM("Gripper status is " << (isGripperActive ? "active":"not active"));					
	}
ros::ServiceClient placeplan_service_client = node_handle.serviceClient<task_planning::PlacePlan>("placeplan");
	// Initialize the gripper command publisher.
	// gripper_command_publisher = node_handle.advertise<std_msgs::String>("GripperCommand",1);

	// gripper_command.data = gripperMode;
	// gripper_command_publisher.publish(gripper_command);
	#endif
}





//#########################################################
//#########################################################
//#########################################################
//This is the function being currently used to reason about grid placements
//The current interface lets us use the placement module to calculate target grid positions, and toggle the behavior of invoking final adjustment corrections using the #USE_PUSHING compiler flag.
//This function invokes low level control logic implemented in MoveitNode that communicates with and moves the robot
//#########################################################
//#########################################################
//#########################################################


// bool TaskPlanner::pick_and_drop(const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP)
// {
// 	// Overloaded function for PICK_AND_PLACE and PICK_AND_MOVE.
// 	ROS_INFO_STREAM(" PERFORMING A PICK AND DROP QUERY");
// 	ROS_WARN_STREAM("drop pose:"<<drop_pose.pose.position.x << ","<<drop_pose.pose.position.y<<","<<drop_pose.pose.position.z);
// 	bool success = false;
// 	bool ADJUSTED_DROP_SUCCESS = true;
// 	int OBJECT_ATTACHMENT = 0;
// 	bool IS_OBJECT_ATTACHED = true;

// 	PlacePlanClient ac("/placeplan_action", true);
// 	ac.waitForServer();
// 	task_planning::PlacePlanGoal placeplan_goal;
//     placeplan_goal.target_object_pose = target_obj_pose;
//     placeplan_goal.target_object_name = object_label;
//     ac.sendGoal(placeplan_goal, &placeplan_doneCb, &placeplan_activeCb, &placeplan_feedbackCb);

//     ros::spinOnce();

// 	ros::ServiceClient clt_pusher;
// 	auto transfer_pose_s = transfer_pose_1;
// 	auto transfer_pose_t = transfer_pose_2;
// 	if(source_bin == "bin_2" && target_bin == "bin_1")
// 	{
// 		transfer_pose_s = transfer_pose_2;
// 		transfer_pose_t = transfer_pose_1;
// 	}	

// 	double z_offset_before_drop = -0.1;
// 	auto sensing_structures = comm->current_sensing_structures();
// 	utilities::writeToLog("Inside pick_and_drop");
// 	if(executeTask("PICK",grasp_point, size))
// 	{
// 		ROS_ERROR_STREAM("\n\n\n pick_and_drop PICKED");
// 		if(REGRASP)
// 		{
// 			success = false;
// 		}
// 		else
// 		{
// 			OBJECT_ATTACHMENT = is_object_attached(source_bin, object_label, grasped_obj_pose);
// 			IS_OBJECT_ATTACHED = OBJECT_ATTACHMENT == 1;
// 			if(IS_OBJECT_ATTACHED)
// 			{

// 				auto retracted_drop = drop_pose.pose;
// 				retracted_drop.position.z-=z_offset_before_drop;


// 				if(planToStartOfPreC("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
// 				{
					
// 					//Delayed release can account for any latency of releasing the suction to preemptively disengage after n(=4.0) seconds.
// 					// delayed_release(4.0);
// 					if(executeTask("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
// 					{
// 						ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM SOURCE TO TARGET");
// 						while(!is_placeplan_ready_);
// 						utilities::writeToLog("Starting adjusted_drop");
// 						ADJUSTED_DROP_SUCCESS = adjusted_drop(target_obj_pose, pre_push_pose_,z_offset_before_drop, target_bin, object_label);
// 						utilities::writeToLog("adjusted_drop result:"+std::to_string(ADJUSTED_DROP_SUCCESS));

// #ifdef USE_PUSHING

// 						if(ADJUSTED_DROP_SUCCESS){
// 							bool safe_move_away = true;
// 							auto safe_move_pose = transfer_pose_t;
// 							sleep(SENSING_SLEEP);
// 							safe_move_pose.pose.orientation = get_current_pose().pose.orientation;
// 							safe_move_pose.pose.position.z = 0.46;

// 							safe_move_away = moveit_node.plan_and_execute_via_waypoints(safe_move_pose);
// 							if(!safe_move_away)
// 							{
// 								safe_move_away = moveit_node.plan_and_execute(safe_move_pose);
// 								ROS_ERROR_STREAM("FAILED TO RETURN OVERHEAD SAFE POSE.");
								
// 							}
// 							if(safe_move_away)
// 							{
// 									moveit_node.addBin();

// 									ros::NodeHandle nh;
// 									utilities::writeToLog("Calling Push Adjustment - inter");
// 									ROS_INFO_STREAM("Hi, Changkyu");
// 									ros::ServiceClient clt_pusher
// 								     = nh.serviceClient<rl_msgs::organizing_pusher_jdxdemo_srv>(
// 								       "/iiwa/changkyu/organizer_jdxdemo/pusher");
// 								    rl_msgs::organizing_pusher_jdxdemo_srv::Request req;
// 								    rl_msgs::organizing_pusher_jdxdemo_srv::Response res;
// 								    req.camera_name = "camera2";
// 								    req.param="thresh=0.01,iter_max=4";
// 								    clt_pusher.call(req, res);    
// 								    ROS_INFO_STREAM("Bye Changkyu");
// 								    utilities::writeToLog("Done with Push Adjustment - inter");						
// 									moveit_node.deleteBin();
									
// 							}
// 						}else{
// 							ROS_ERROR_STREAM("\n\n\n !!!pick_and_drop adjusted drop failed");
// 						}
// #endif
// 						sleep(SENSING_SLEEP);
// 						ROS_ERROR_STREAM("ADJUSTED_DROP_SUCCESS:"<<ADJUSTED_DROP_SUCCESS);
// 						comm->set_sensing(std::get<0>(sensing_structures), std::get<1>(sensing_structures));
// 						sleep(SENSING_SLEEP);
// 						comm->start_sensing();
// 						if(planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP) && executeTask("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP))
// 						{
// 							ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM TARGET BACK TO SOURCE");
// 							success = true;
// 						}
// 						else
// 						{
// 							utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");	
// 							ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");
// 						}
// 						if(!ADJUSTED_DROP_SUCCESS)
// 						{

// 							ROS_ERROR_STREAM("\n\n\nFailed to successfully complete ADJUSTED DROP");
// 							executeTask("RELEASE");
// 							return false;
// 						}
// 					}
// 					else
// 					{
// 						utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
// 						ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
// 					}
// 				}
// 			}
// 			else
// 			{
// 				utilities::writeToLog("pick_and_drop OBJECT NOT ATTACHED");	
// 				ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE OBJECT NOT ATTACHED");
// 			}
// 		}
// 	}
// 	else
// 	{
// 		utilities::writeToLog("Failed to go to pick pose");
// 		ROS_ERROR_STREAM("\n\n\nFAILED TO PICK");
// 	}

// 	if(!success)
// 	{
// 		transfer_pose_s = transfer_pose_1;
// 		transfer_pose_t = transfer_pose_2;
// 		if(source_bin == "bin_2" && target_bin == "bin_1")
// 		{
// 			transfer_pose_s = transfer_pose_2;
// 			transfer_pose_t = transfer_pose_1;
// 		}

// 		auto release_pose = transfer_pose_s;
// 		release_pose.pose.position.z-=0.08;

// 		// delayed_release(2);

// 		bool OBJECT_GRASPED_FROM_WRONG_FACE = !IS_OBJECT_ATTACHED && OBJECT_ATTACHMENT == -1;
// 		bool REGRASP_REQUIRED_FROM_DETECTION = REGRASP;
// 		if( OBJECT_GRASPED_FROM_WRONG_FACE || REGRASP_REQUIRED_FROM_DETECTION )
// 		{
// 			utilities::writeToLog("REGRASP");
// 			topple(source_bin, object_label);
// 			utilities::writeToLog("Finish REGRASP");
// 		}


// 		auto intermediate_release = get_current_pose().pose;
// 		intermediate_release.position.z+=0.1;
// 		moveit_node.plan_and_execute_via_waypoints(intermediate_release);

// 		if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
// 		{
// 			moveit_node.plan_and_execute(transfer_pose_s);
// 			ROS_ERROR_STREAM("FAILED TO RETURN TO A SAFE POSE.");
			
// 		}

// 		executeTask("RELEASE");
// 	}
// 	utilities::writeToLog("Finish pick_and_drop "+std::to_string(success));
// 	return success;
// }


// //#########################################################
// //#########################################################
// //#########################################################












bool TaskPlanner::pick_and_drop(const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP)
{
	// Overloaded function for PICK_AND_PLACE and PICK_AND_MOVE.
	ROS_INFO_STREAM(" PERFORMING A PICK AND DROP QUERY");
	ROS_WARN_STREAM("drop pose:"<<drop_pose.pose.position.x << ","<<drop_pose.pose.position.y<<","<<drop_pose.pose.position.z);
	bool success = false;
	bool ADJUSTED_DROP_SUCCESS = true;
	int OBJECT_ATTACHMENT = 0;
	bool IS_OBJECT_ATTACHED = true;

	PlacePlanClient ac("/placeplan_action", true);
	ac.waitForServer();
	task_planning::PlacePlanGoal placeplan_goal;
    placeplan_goal.target_object_pose = target_obj_pose;
    placeplan_goal.target_object_name = "dove";
    ac.sendGoal(placeplan_goal, &placeplan_doneCb, &placeplan_activeCb, &placeplan_feedbackCb);

    ros::spinOnce();

	ros::ServiceClient clt_pusher;
	auto transfer_pose_s = transfer_pose_1;
	auto transfer_pose_t = transfer_pose_2;
	if(source_bin == "bin_2" && target_bin == "bin_1")
	{
		transfer_pose_s = transfer_pose_2;
		transfer_pose_t = transfer_pose_1;
	}	

	double z_offset_before_drop = -0.15;
	auto sensing_structures = comm->current_sensing_structures();
	utilities::writeToLog("Inside pick_and_drop");
	if(executeTask("PICK",grasp_point, size))
	{
		ROS_ERROR_STREAM("\n\n\n pick_and_drop PICKED");
		if(REGRASP)
		{
			success = false;
		}
		else
		{
			OBJECT_ATTACHMENT = is_object_attached(source_bin, object_label, grasped_obj_pose);
			IS_OBJECT_ATTACHED = OBJECT_ATTACHMENT == 1;
			if(IS_OBJECT_ATTACHED)
			{

				// auto retracted_drop = drop_pose.pose;
				auto retracted_drop = transfer_pose_t.pose;
				retracted_drop.orientation = drop_pose.pose.orientation;
				retracted_drop.position.z = drop_pose.pose.position.z;
				// retracted_drop.position.y -= z_offset_before_drop;
				// retracted_drop.position.x -= z_offset_before_drop;
				retracted_drop.position.z-=z_offset_before_drop;


				if(planToStartOfPreC("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
				{
					
					//Delayed release can account for any latency of releasing the suction to preemptively disengage after n(=4.0) seconds.
					// delayed_release(4.0);
					if(executeTask("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
					{
						ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM SOURCE TO TARGET");
						while(!is_placeplan_ready_);
						utilities::writeToLog("Starting adjusted_drop");
						ADJUSTED_DROP_SUCCESS = adjusted_drop(target_obj_pose, pre_push_pose_,z_offset_before_drop, target_bin, object_label);
						utilities::writeToLog("adjusted_drop result:"+std::to_string(ADJUSTED_DROP_SUCCESS));

#ifdef USE_PUSHING

						if(ADJUSTED_DROP_SUCCESS){
							bool safe_move_away = true;
							auto safe_move_pose = transfer_pose_t;
							sleep(SENSING_SLEEP);
							safe_move_pose.pose.orientation = get_current_pose().pose.orientation;
							safe_move_pose.pose.position.z = 0.46;

							safe_move_away = moveit_node.plan_and_execute_via_waypoints(safe_move_pose);
							if(!safe_move_away)
							{
								safe_move_away = moveit_node.plan_and_execute(safe_move_pose);
								ROS_ERROR_STREAM("FAILED TO RETURN OVERHEAD SAFE POSE.");
								
							}
							if(safe_move_away)
							{
									moveit_node.addBin();

									ros::NodeHandle nh;
									utilities::writeToLog("Calling Push Adjustment - inter");
									ROS_INFO_STREAM("Hi, Changkyu");
									ros::ServiceClient clt_pusher
								     = nh.serviceClient<rl_msgs::organizing_pusher_jdxdemo_srv>(
								       "/iiwa/changkyu/organizer_jdxdemo/pusher");
								    rl_msgs::organizing_pusher_jdxdemo_srv::Request req;
								    rl_msgs::organizing_pusher_jdxdemo_srv::Response res;
								    req.camera_name = "camera2";
								    req.param="thresh=0.015,iter_max=4";
								    clt_pusher.call(req, res);    
								    ROS_INFO_STREAM("Bye Changkyu");
								    utilities::writeToLog("Done with Push Adjustment - inter");						
									moveit_node.deleteBin();
									
							}
						}else{
							ROS_ERROR_STREAM("\n\n\n !!!pick_and_drop adjusted drop failed");
						}
#endif
						sleep(SENSING_SLEEP);
						ROS_ERROR_STREAM("ADJUSTED_DROP_SUCCESS:"<<ADJUSTED_DROP_SUCCESS);
						comm->set_sensing(std::get<0>(sensing_structures), std::get<1>(sensing_structures));
						sleep(SENSING_SLEEP);
						comm->start_sensing();
						if(planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP) && executeTask("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP))
						{
							ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM TARGET BACK TO SOURCE");
							success = true;
						}
						else
						{
							utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");	
							ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");
						}
						if(!ADJUSTED_DROP_SUCCESS)
						{

							ROS_ERROR_STREAM("\n\n\nFailed to successfully complete ADJUSTED DROP");
							executeTask("RELEASE");
							return false;
						}
					}
					else
					{
						utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
						ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
					}
				}
			}
			else
			{
				utilities::writeToLog("pick_and_drop OBJECT NOT ATTACHED");	
				ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE OBJECT NOT ATTACHED");
			}
		}
	}
	else
	{
		utilities::writeToLog("Failed to go to pick pose");
		ROS_ERROR_STREAM("\n\n\nFAILED TO PICK");
	}

	if(!success)
	{
		transfer_pose_s = transfer_pose_1;
		transfer_pose_t = transfer_pose_2;
		if(source_bin == "bin_2" && target_bin == "bin_1")
		{
			transfer_pose_s = transfer_pose_2;
			transfer_pose_t = transfer_pose_1;
		}

		auto release_pose = transfer_pose_s;
		release_pose.pose.position.z-=0.08;

		// delayed_release(2);

		bool OBJECT_GRASPED_FROM_WRONG_FACE = !IS_OBJECT_ATTACHED && OBJECT_ATTACHMENT == -1;
		bool REGRASP_REQUIRED_FROM_DETECTION = REGRASP;
		if( OBJECT_GRASPED_FROM_WRONG_FACE || REGRASP_REQUIRED_FROM_DETECTION )
		{
			utilities::writeToLog("REGRASP");
			topple(source_bin, object_label);
			utilities::writeToLog("Finish REGRASP");
		}


		auto intermediate_release = get_current_pose().pose;
		intermediate_release.position.z+=0.1;
		moveit_node.plan_and_execute_via_waypoints(intermediate_release);

		if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
		{
			moveit_node.plan_and_execute(transfer_pose_s);
			ROS_ERROR_STREAM("FAILED TO RETURN TO A SAFE POSE.");
			
		}

		executeTask("RELEASE");
		sleep(2*SENSING_SLEEP);
	}
	utilities::writeToLog("Finish pick_and_drop "+std::to_string(success));


	return success;
}

bool TaskPlanner::pick_and_drop_with_tightenv(const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP)
{
	// Overloaded function for PICK_AND_PLACE and PICK_AND_MOVE.
	ROS_INFO_STREAM(" PERFORMING A PICK AND DROP QUERY");
	ROS_WARN_STREAM("drop pose:"<<drop_pose.pose.position.x << ","<<drop_pose.pose.position.y<<","<<drop_pose.pose.position.z);
	bool success = false;
	bool ADJUSTED_DROP_SUCCESS = true;
	int OBJECT_ATTACHMENT = 0;
	bool IS_OBJECT_ATTACHED = true;

	PlacePlanMultiClient ac("/placeplanmulti_action", true);
	ac.waitForServer();
	task_planning::PlacePlanMultiGoal placeplanmulti_goal;
    placeplanmulti_goal.target_object_pose = target_obj_pose;
    placeplanmulti_goal.target_object_name = "dove";
    ac.sendGoal(placeplanmulti_goal, &placeplanmulti_doneCb, &placeplanmulti_activeCb, &placeplanmulti_feedbackCb);

    ros::spinOnce();

	ros::ServiceClient clt_pusher;
	auto transfer_pose_s = transfer_pose_1;
	auto transfer_pose_t = transfer_pose_2;
	if(source_bin == "bin_2" && target_bin == "bin_1")
	{
		transfer_pose_s = transfer_pose_2;
		transfer_pose_t = transfer_pose_1;
	}	

	double z_offset_before_drop = -0.15;
	auto sensing_structures = comm->current_sensing_structures();
	utilities::writeToLog("Inside pick_and_drop");
	if(executeTask("PICK",grasp_point, size))
	{
		ROS_ERROR_STREAM("\n\n\n pick_and_drop PICKED");
		if(REGRASP)
		{
			success = false;
		}
		else
		{
			OBJECT_ATTACHMENT = is_object_attached(source_bin, object_label, grasped_obj_pose);
			IS_OBJECT_ATTACHED = OBJECT_ATTACHMENT == 1;
			if(IS_OBJECT_ATTACHED)
			{

				// auto retracted_drop = drop_pose.pose;
				auto retracted_drop = transfer_pose_t.pose;
				retracted_drop.orientation = drop_pose.pose.orientation;
				retracted_drop.position.z = drop_pose.pose.position.z;
				// retracted_drop.position.y -= z_offset_before_drop;
				// retracted_drop.position.x -= z_offset_before_drop;
				retracted_drop.position.z-=z_offset_before_drop;


				if(planToStartOfPreC("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
				{
					
					//Delayed release can account for any latency of releasing the suction to preemptively disengage after n(=4.0) seconds.
					// delayed_release(4.0);
					if(executeTask("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
					{
						ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM SOURCE TO TARGET");
						while(!is_placeplanmulti_ready_);
						utilities::writeToLog("Starting adjusted_drop");
						ADJUSTED_DROP_SUCCESS = adjusted_drop_with_tightenv(target_obj_pose, pre_push_pose_list_,z_offset_before_drop, target_bin, object_label);
						utilities::writeToLog("adjusted_drop result:"+std::to_string(ADJUSTED_DROP_SUCCESS));

#ifdef USE_PUSHING

						if(ADJUSTED_DROP_SUCCESS){
							bool safe_move_away = true;
							auto safe_move_pose = transfer_pose_t;
							sleep(SENSING_SLEEP);
							safe_move_pose.pose.orientation = get_current_pose().pose.orientation;
							safe_move_pose.pose.position.z = 0.46;

							safe_move_away = moveit_node.plan_and_execute_via_waypoints(safe_move_pose);
							if(!safe_move_away)
							{
								safe_move_away = moveit_node.plan_and_execute(safe_move_pose);
								ROS_ERROR_STREAM("FAILED TO RETURN OVERHEAD SAFE POSE.");
								
							}
							if(safe_move_away)
							{
									moveit_node.addBin();

									ros::NodeHandle nh;
									utilities::writeToLog("Calling Push Adjustment - inter");
									ROS_INFO_STREAM("Hi, Changkyu");
									ros::ServiceClient clt_pusher
								     = nh.serviceClient<rl_msgs::organizing_pusher_jdxdemo_srv>(
								       "/iiwa/changkyu/organizer_jdxdemo/pusher");
								    rl_msgs::organizing_pusher_jdxdemo_srv::Request req;
								    rl_msgs::organizing_pusher_jdxdemo_srv::Response res;
								    req.camera_name = "camera2";
								    req.param="thresh=0.015,iter_max=4";
								    clt_pusher.call(req, res);    
								    ROS_INFO_STREAM("Bye Changkyu");
								    utilities::writeToLog("Done with Push Adjustment - inter");						
									moveit_node.deleteBin();
									
							}
						}else{
							ROS_ERROR_STREAM("\n\n\n !!!pick_and_drop adjusted drop failed");
						}
#endif
						sleep(SENSING_SLEEP);
						ROS_ERROR_STREAM("ADJUSTED_DROP_SUCCESS:"<<ADJUSTED_DROP_SUCCESS);
						comm->set_sensing(std::get<0>(sensing_structures), std::get<1>(sensing_structures));
						sleep(SENSING_SLEEP);
						comm->start_sensing();
						if(planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP) && executeTask("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP))
						{
							ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM TARGET BACK TO SOURCE");
							success = true;
						}
						else
						{
							utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");	
							ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");
						}
						if(!ADJUSTED_DROP_SUCCESS)
						{

							ROS_ERROR_STREAM("\n\n\nFailed to successfully complete ADJUSTED DROP");
							executeTask("RELEASE");
							return false;
						}
					}
					else
					{
						utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
						ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
					}
				}
			}
			else
			{
				utilities::writeToLog("pick_and_drop OBJECT NOT ATTACHED");	
				ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE OBJECT NOT ATTACHED");
			}
		}
	}
	else
	{
		utilities::writeToLog("Failed to go to pick pose");
		ROS_ERROR_STREAM("\n\n\nFAILED TO PICK");
	}

	if(!success)
	{
		transfer_pose_s = transfer_pose_1;
		transfer_pose_t = transfer_pose_2;
		if(source_bin == "bin_2" && target_bin == "bin_1")
		{
			transfer_pose_s = transfer_pose_2;
			transfer_pose_t = transfer_pose_1;
		}

		auto release_pose = transfer_pose_s;
		release_pose.pose.position.z-=0.08;

		// delayed_release(2);

		bool OBJECT_GRASPED_FROM_WRONG_FACE = !IS_OBJECT_ATTACHED && OBJECT_ATTACHMENT == -1;
		bool REGRASP_REQUIRED_FROM_DETECTION = REGRASP;
		if( OBJECT_GRASPED_FROM_WRONG_FACE || REGRASP_REQUIRED_FROM_DETECTION )
		{
			utilities::writeToLog("REGRASP");
			topple(source_bin, object_label);
			utilities::writeToLog("Finish REGRASP");
		}


		auto intermediate_release = get_current_pose().pose;
		intermediate_release.position.z+=0.1;
		moveit_node.plan_and_execute_via_waypoints(intermediate_release);

		if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
		{
			moveit_node.plan_and_execute(transfer_pose_s);
			ROS_ERROR_STREAM("FAILED TO RETURN TO A SAFE POSE.");
			
		}

		executeTask("RELEASE");
		sleep(2*SENSING_SLEEP);
	}
	utilities::writeToLog("Finish pick_and_drop "+std::to_string(success));


	return success;
}

//#########################################################
//#########################################################
//#########################################################






















void TaskPlanner::test_joint_velocity(){
	//command_velocity_.velocity.a1 = -1.3;
	// command_velocity_.velocity.a1 = -0.008;

	// command_velocity_.velocity.a2 = 0;
	// command_velocity_.velocity.a3 = 0;
	// command_velocity_.velocity.a4 = 0;
	// command_velocity_.velocity.a5 = 0;
	// command_velocity_.velocity.a6 = 0;
	// command_velocity_.velocity.a7 = 0;
	// my_iiwa.getPathParametersService().setJointRelativeVelocity(1);
	// for(int i = 0; i < 14; i ++){
	// 	my_iiwa.setJointVelocity(command_velocity_);
	// 	ros::Duration(0.1).sleep();
	// 	command_velocity_.velocity.a1 -= 0.1;
	// }
	// 	my_iiwa.setJointVelocity(command_velocity_);
	// 	ros::Duration(1).sleep();
	
	
	// command_velocity_.velocity.a1 = 0;
	// my_iiwa.setJointVelocity(command_velocity_);
	moveit_node.test_joint_velocity();


}

void TaskPlanner::getGripperStatus()
{
	ROS_INFO_STREAM("Getting gripper status..."<<gripperStatusMsg->gIMC<<" | "<<gripperStatusMsg->gSTA);
	if (gripperStatusMsg->gIMC == 3 && gripperStatusMsg->gSTA == 3)
	{
		isGripperActive = true;
	}
}

void TaskPlanner::delayed_release_event(const ros::TimerEvent& event)
{
	executeTask("RELEASE");
	ROS_ERROR_STREAM("\nDELAYED RELEASE\n\nDELAYED RELEASE\n\nDELAYED RELEASE\n\nDELAYED RELEASE\n");
}
void TaskPlanner::delayed_release(double delay)
{
	release_timer = node_handle.createTimer(ros::Duration(delay), &TaskPlanner::delayed_release_event, this, true);
}

bool TaskPlanner::executeTask(std::string query,const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& target_pose, double size, std::string source_bin, std::string target_bin)
{
	// Overloaded function for PICK_AND_PLACE and PICK_AND_MOVE.
	ROS_INFO_STREAM("Received a " << query << " query...");
	
	
	if (query == "PICK_AND_PLACE")
	{

		bool success = false;
		auto transfer_pose_s = transfer_pose_1;
		auto transfer_pose_t = transfer_pose_2;
		if(source_bin == "bin_2" && target_bin == "bin_1")
		{
			transfer_pose_s = transfer_pose_2;
			transfer_pose_t = transfer_pose_1;
		}	
		
		if(executeTask("PICK",grasp_point, size))
		{
			ROS_ERROR_STREAM("PICKED");
			if(planToStartOfPreC("TRANSFER", source_bin, target_bin, (int)PREGRASP_TO_DROP))
			{
				if(is_object_attached(source_bin))
				{
					delayed_release(4.0);
					if(executeTask("TRANSFER", source_bin, target_bin, (int)PREGRASP_TO_DROP))
					{
						ROS_ERROR_STREAM("MOVED FROM SOURCE TO TARGET");
						
						// executeTask("RELEASE");
						comm->start_sensing();
						if(planToStartOfPreC("TRANSFER", target_bin, source_bin, (int)DROP_TO_PREGRASP) && executeTask("TRANSFER", target_bin, source_bin, (int)DROP_TO_PREGRASP))
						{
							ROS_ERROR_STREAM("MOVED FROM TARGET BACK TO SOURCE");
							success = true;
						}
						else
						{
							ROS_ERROR_STREAM("FAILED TO EXECUTE PRECOMPUTATION BACK");
						}
					}
					else
					{
						ROS_ERROR_STREAM("FAILED TO EXECUTE PRECOMPUTATION FWD");
					}
				}
			}
		}
		else
		{
			ROS_ERROR_STREAM("FAILED TO PICK");
		}

		if(!success)
		{
			executeTask("PLACE",transfer_pose_s);
		}

		return success;
	}
}


bool TaskPlanner::pick_and_drop_without_pose_estimation(const geometry_msgs::PoseStamped& grasp_point, geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP)
{
	// Overloaded function for PICK_AND_PLACE and PICK_AND_MOVE.
	ROS_INFO_STREAM(" PERFORMING A PICK AND DROP QUERY");
	ROS_WARN_STREAM("drop pose:"<<drop_pose.pose.position.x << ","<<drop_pose.pose.position.y<<","<<drop_pose.pose.position.z);
	bool success = false;
	bool ADJUSTED_DROP_SUCCESS = true;
	int OBJECT_ATTACHMENT = 0;
	bool IS_OBJECT_NOT_ATTACHED = false;
	bool IS_OBJECT_ATTACHED = true;

	PlacePlanClient ac("/placeplan_action", true);
	//ac.waitForServer();
	task_planning::PlacePlanGoal placeplan_goal;
    placeplan_goal.target_object_pose = target_obj_pose;
    placeplan_goal.target_object_name = "dove";
    //ac.sendGoal(placeplan_goal, &placeplan_doneCb, &placeplan_activeCb, &placeplan_feedbackCb);


/*
	task_planning::PlacePlan srv;
	ros::ServiceClient placeplan_service_client = node_handle.serviceClient<task_planning::PlacePlan>("/placeplan_service");
	srv.request.target_object_pose = target_obj_pose;
	// srv.request.target_object_pose.position.x += X_CALIBRATION_OFFSET;
	// srv.request.target_object_pose.position.y += Y_CALIBRATION_OFFSET;

	srv.request.target_object_name = object_label;
	if (placeplan_service_client.call(srv))
	{
		pre_push_pose_  = srv.response.pre_push_pose;
		post_push_pose_ = srv.response.post_push_pose;
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
*/
    ros::spinOnce();

	ros::ServiceClient clt_pusher;
	auto transfer_pose_s = transfer_pose_1;
	auto transfer_pose_t = transfer_pose_2;
	if(source_bin == "bin_2" && target_bin == "bin_1")
	{
		transfer_pose_s = transfer_pose_2;
		transfer_pose_t = transfer_pose_1;
	}	
	//Toggle behavior of going to the center versus going over grid
	// transfer_pose_t.pose.position.z-=0.25;
	// double z_offset_before_drop = -0.15;
	
	// transfer_pose_t = drop_pose;

	// double z_offset_before_drop = -0.1;
	drop_pose.pose.orientation = transfer_pose_t.pose.orientation;
		drop_pose.pose.orientation.x = 0.34554;
	drop_pose.pose.orientation.y = 0.93840;
	drop_pose.pose.orientation.z = -0.001849;
	drop_pose.pose.orientation.w = -0.000465;
	double z_offset_before_drop = -0.06;
	auto sensing_structures = comm->current_sensing_structures();
	utilities::writeToLog("Inside pick_and_drop");
	if(executeTask("PICK",grasp_point, size))
	{
		ROS_ERROR_STREAM("\n\n\n pick_and_drop PICKED");
		// if(REGRASP)
		// {
		// 	success = false;
		// }
		// else
		{
			OBJECT_ATTACHMENT = is_object_attached(source_bin, object_label, grasped_obj_pose, false);
			IS_OBJECT_ATTACHED = OBJECT_ATTACHMENT == 1;
			IS_OBJECT_NOT_ATTACHED = OBJECT_ATTACHMENT == 0;
			if(!IS_OBJECT_NOT_ATTACHED)
			{

				auto retracted_drop = drop_pose.pose;
				retracted_drop.position.z-=z_offset_before_drop;


				auto viewpoint_pose = transfer_pose_t.pose;
				viewpoint_pose.orientation = drop_pose.pose.orientation;
				viewpoint_pose.position.z=retracted_drop.position.z;

				// if(planToStartOfPreC("TRANSFER", source_bin, target_bin, viewpoint_pose, (int)PREGRASP_TO_DROP))
				if(planToStartOfPreC("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
				{
				
					// delayed_release(4.0);
					// if(executeTask("TRANSFER", source_bin, target_binp, viewpoint_pose, (int)PREGRASP_TO_DROP))
					if(executeTask("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
					{
						ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM SOURCE TO TARGET");
						// while(!is_placeplan_ready_);
						//Sense, offset, move, drop
						// std::string input;
						// std::cin>>input;
						utilities::writeToLog("Starting adjusted_drop");
						ADJUSTED_DROP_SUCCESS = true;//adjusted_drop(target_obj_pose, pre_push_pose_,z_offset_before_drop, target_bin, object_label);
						utilities::writeToLog("adjusted_drop result:"+std::to_string(ADJUSTED_DROP_SUCCESS));
						executeTask("RELEASE");
						moveit_node.plan_and_execute_via_waypoints(0,0,-z_offset_before_drop);

						sleep(SENSING_SLEEP);
						ROS_ERROR_STREAM("ADJUSTED_DROP_SUCCESS:"<<ADJUSTED_DROP_SUCCESS);
						comm->set_sensing(std::get<0>(sensing_structures), std::get<1>(sensing_structures));
						sleep(SENSING_SLEEP);
						//ROS_ERROR_STREAM("Waiting for the input");
						// std::string xxx;
						// std::cin>>xxx;
						comm->start_sensing();
						if(planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP) && executeTask("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP))
						{
							ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM TARGET BACK TO SOURCE");
							success = true;
						}
						else
						{
							utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");	
							ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");
						}
						if(!ADJUSTED_DROP_SUCCESS)
						{

							ROS_ERROR_STREAM("\n\n\nFailed to successfully complete ADJUSTED DROP");
							executeTask("RELEASE");
							return false;
						}
					}
					else
					{
						utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
						ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
					}
				}
			}
			else
			{
				utilities::writeToLog("pick_and_drop OBJECT NOT ATTACHED");	
				ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE OBJECT NOT ATTACHED");
			}
		}
	}
	else
	{
		utilities::writeToLog("Failed to go to pick pose");
		ROS_ERROR_STREAM("\n\n\nFAILED TO PICK");
	}

	if(!success)
	{
		transfer_pose_s = transfer_pose_1;
		transfer_pose_t = transfer_pose_2;
		if(source_bin == "bin_2" && target_bin == "bin_1")
		{
			transfer_pose_s = transfer_pose_2;
			transfer_pose_t = transfer_pose_1;
		}

		auto release_pose = transfer_pose_s;
		release_pose.pose.position.z-=0.08;

		// delayed_release(2);

		bool OBJECT_GRASPED_FROM_WRONG_FACE = !IS_OBJECT_ATTACHED && OBJECT_ATTACHMENT == -1;
		bool REGRASP_REQUIRED_FROM_DETECTION = REGRASP;
		if( OBJECT_GRASPED_FROM_WRONG_FACE || REGRASP_REQUIRED_FROM_DETECTION )
		{
			// // auto current_pose = get_current_pose().pose;
			// // current_pose.pose.position.x = 0.39;
			// // current_pose.pose.position.y = 0.34;
			// // current_pose.pose.position.z = 0.55;
			// ROS_ERROR_STREAM("ATTEMPTING TO RETURN TO A RETRACTED POSE.");

			// // release_pose.pose.orientation.w = 0.087;
			// // release_pose.pose.orientation.x = 0.008;
			// // release_pose.pose.orientation.y = 0.992;
			// // release_pose.pose.orientation.z = 0.087;
			// release_pose.pose.orientation.w = -0.130;
			// release_pose.pose.orientation.x = -0.011;
			// release_pose.pose.orientation.y = -0.988;
			// release_pose.pose.orientation.z = -0.086;
			// release_pose.pose.position.x-=0.05;


			// // if(!executeTask("PLACE",transfer_pose_s))
			// auto retracted_pose = get_current_pose();
			// retracted_pose.pose.position.z+=0.1;
			// moveit_node.plan_and_execute_via_waypoints(retracted_pose);
			// utilities::writeToLog("REGRASP");
			// retracted_pose.pose.position = release_pose.pose.position;
			// moveit_node.plan_and_execute_via_waypoints(retracted_pose);
			// topple(source_bin, object_label);
			// utilities::writeToLog("Finish REGRASP");
		}


		
		// if(!moveit_node.plan_and_execute_via_waypoints(release_pose))
		// {
		// 	moveit_node.plan_and_execute(release_pose);
		// 	// executeTask("RELEASE");
		// 	ROS_ERROR_STREAM("FAILED TO RETURN TO A RETRACTED POSE.");
			
		// }

		auto intermediate_release = get_current_pose().pose;
		intermediate_release.position.z+=0.1;
		moveit_node.plan_and_execute_via_waypoints(intermediate_release);

		if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
		{
			moveit_node.plan_and_execute(transfer_pose_s);
			// executeTask("RELEASE");
			ROS_ERROR_STREAM("FAILED TO RETURN TO A SAFE POSE.");
			
		}
		executeTask("RELEASE");
	}
	utilities::writeToLog("Finish pick_and_drop "+std::to_string(success));
	return success;
}

bool TaskPlanner::pick_and_drop_without_topple(const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP)
{
	// Overloaded function for PICK_AND_PLACE and PICK_AND_MOVE.
	ROS_INFO_STREAM(" PERFORMING A PICK AND DROP QUERY");
	ROS_WARN_STREAM("drop pose:"<<drop_pose.pose.position.x << ","<<drop_pose.pose.position.y<<","<<drop_pose.pose.position.z);
	bool success = false;
	bool ADJUSTED_DROP_SUCCESS = true;
	int OBJECT_ATTACHMENT = 0;
	bool IS_OBJECT_ATTACHED = true;

	PlacePlanClient ac("/placeplan_action", true);
	ac.waitForServer();
	task_planning::PlacePlanGoal placeplan_goal;
    placeplan_goal.target_object_pose = target_obj_pose;
    placeplan_goal.target_object_name = "dove";
    ac.sendGoal(placeplan_goal, &placeplan_doneCb, &placeplan_activeCb, &placeplan_feedbackCb);


/*
	task_planning::PlacePlan srv;
	ros::ServiceClient placeplan_service_client = node_handle.serviceClient<task_planning::PlacePlan>("/placeplan_service");
	srv.request.target_object_pose = target_obj_pose;
	// srv.request.target_object_pose.position.x += X_CALIBRATION_OFFSET;
	// srv.request.target_object_pose.position.y += Y_CALIBRATION_OFFSET;

	srv.request.target_object_name = object_label;
	if (placeplan_service_client.call(srv))
	{
		pre_push_pose_  = srv.response.pre_push_pose;
		post_push_pose_ = srv.response.post_push_pose;
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
*/
    ros::spinOnce();

	ros::ServiceClient clt_pusher;
	auto transfer_pose_s = transfer_pose_1;
	auto transfer_pose_t = transfer_pose_2;
	if(source_bin == "bin_2" && target_bin == "bin_1")
	{
		transfer_pose_s = transfer_pose_2;
		transfer_pose_t = transfer_pose_1;
	}	
	//Toggle behavior of going to the center versus going over grid
	// transfer_pose_t.pose.position.z-=0.25;
	// double z_offset_before_drop = -0.15;
	
	// transfer_pose_t = drop_pose;

	// double z_offset_before_drop = -0.1;

	double z_offset_before_drop = -0.1;
	auto sensing_structures = comm->current_sensing_structures();
	utilities::writeToLog("Inside pick_and_drop");
	if(executeTask("PICK",grasp_point, size))
	{
		ROS_ERROR_STREAM("\n\n\n pick_and_drop PICKED");
		if(REGRASP)
		{
			success = false;
		}
		else
		{
			OBJECT_ATTACHMENT = is_object_attached(source_bin, object_label, grasped_obj_pose);
			IS_OBJECT_ATTACHED = OBJECT_ATTACHMENT == 1;
			if(IS_OBJECT_ATTACHED)
			{

				auto retracted_drop = drop_pose.pose;
				retracted_drop.position.z-=z_offset_before_drop;

				auto viewpoint_pose = transfer_pose_t.pose;
				viewpoint_pose.orientation = drop_pose.pose.orientation;
				viewpoint_pose.position.z=retracted_drop.position.z;

				// if(planToStartOfPreC("TRANSFER", source_bin, target_bin, viewpoint_pose, (int)PREGRASP_TO_DROP))
				if(planToStartOfPreC("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
				{
				
					// delayed_release(4.0);
					// if(executeTask("TRANSFER", source_bin, target_binp, viewpoint_pose, (int)PREGRASP_TO_DROP))
					if(executeTask("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
					{
						ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM SOURCE TO TARGET");
						while(!is_placeplan_ready_);
						//Sense, offset, move, drop
						// std::string input;
						// std::cin>>input;
						utilities::writeToLog("Starting adjusted_drop");
						ADJUSTED_DROP_SUCCESS = adjusted_drop(target_obj_pose, pre_push_pose_,z_offset_before_drop, target_bin, object_label);
						utilities::writeToLog("adjusted_drop result:"+std::to_string(ADJUSTED_DROP_SUCCESS));
						// executeTask("RELEASE");

#ifdef USE_PUSHING

						// planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP);
						if(ADJUSTED_DROP_SUCCESS){
							bool safe_move_away = true;
							auto safe_move_pose = transfer_pose_t;
							sleep(SENSING_SLEEP);
							safe_move_pose.pose.orientation = get_current_pose().pose.orientation;
							safe_move_pose.pose.position.z = 0.46;

							safe_move_away = moveit_node.plan_and_execute_via_waypoints(safe_move_pose);
							if(!safe_move_away)
							{
								safe_move_away = moveit_node.plan_and_execute(safe_move_pose);
								// executeTask("RELEASE");
								ROS_ERROR_STREAM("FAILED TO RETURN OVERHEAD SAFE POSE.");
								
							}
							if(safe_move_away)
							{
									moveit_node.addBin();

									ros::NodeHandle nh;
									utilities::writeToLog("Calling Push Adjustment - inter");
									ROS_INFO_STREAM("Hi, Changkyu");
									ros::ServiceClient clt_pusher
								     = nh.serviceClient<rl_msgs::organizing_pusher_jdxdemo_srv>(
								       "/iiwa/changkyu/organizer_jdxdemo/pusher");
								    rl_msgs::organizing_pusher_jdxdemo_srv::Request req;
								    rl_msgs::organizing_pusher_jdxdemo_srv::Response res;
								    req.camera_name = "camera2";
								    req.param="thresh=0.01,iter_max=4";
								    clt_pusher.call(req, res);    
								    ROS_INFO_STREAM("Bye Changkyu");
								    utilities::writeToLog("Done with Push Adjustment - inter");						
									moveit_node.deleteBin();
									
							}
						}else{
							ROS_ERROR_STREAM("\n\n\n !!!pick_and_drop adjusted drop failed");
						}
#endif
						sleep(SENSING_SLEEP);
						ROS_ERROR_STREAM("ADJUSTED_DROP_SUCCESS:"<<ADJUSTED_DROP_SUCCESS);
						comm->set_sensing(std::get<0>(sensing_structures), std::get<1>(sensing_structures));
						sleep(SENSING_SLEEP);
						//ROS_ERROR_STREAM("Waiting for the input");
						// std::string xxx;
						// std::cin>>xxx;
						comm->start_sensing();
						if(planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP) && executeTask("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP))
						{
							ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM TARGET BACK TO SOURCE");
							success = true;
						}
						else
						{
							utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");	
							ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");
						}
						if(!ADJUSTED_DROP_SUCCESS)
						{

							ROS_ERROR_STREAM("\n\n\nFailed to successfully complete ADJUSTED DROP");
							executeTask("RELEASE");
							return false;
						}
					}
					else
					{
						utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
						ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
					}
				}
			}
			else
			{
				utilities::writeToLog("pick_and_drop OBJECT NOT ATTACHED");	
				ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE OBJECT NOT ATTACHED");
			}
		}
	}
	else
	{
		utilities::writeToLog("Failed to go to pick pose");
		ROS_ERROR_STREAM("\n\n\nFAILED TO PICK");
	}

	if(!success)
	{
		transfer_pose_s = transfer_pose_1;
		transfer_pose_t = transfer_pose_2;
		if(source_bin == "bin_2" && target_bin == "bin_1")
		{
			transfer_pose_s = transfer_pose_2;
			transfer_pose_t = transfer_pose_1;
		}

		auto release_pose = transfer_pose_s;
		release_pose.pose.position.z-=0.08;

		// delayed_release(2);

		bool OBJECT_GRASPED_FROM_WRONG_FACE = !IS_OBJECT_ATTACHED && OBJECT_ATTACHMENT == -1;
		bool REGRASP_REQUIRED_FROM_DETECTION = REGRASP;
		if( OBJECT_GRASPED_FROM_WRONG_FACE || REGRASP_REQUIRED_FROM_DETECTION )
		{
			// // auto current_pose = get_current_pose().pose;
			// // current_pose.pose.position.x = 0.39;
			// // current_pose.pose.position.y = 0.34;
			// // current_pose.pose.position.z = 0.55;
			// ROS_ERROR_STREAM("ATTEMPTING TO RETURN TO A RETRACTED POSE.");

			// // release_pose.pose.orientation.w = 0.087;
			// // release_pose.pose.orientation.x = 0.008;
			// // release_pose.pose.orientation.y = 0.992;
			// // release_pose.pose.orientation.z = 0.087;
			// release_pose.pose.orientation.w = -0.130;
			// release_pose.pose.orientation.x = -0.011;
			// release_pose.pose.orientation.y = -0.988;
			// release_pose.pose.orientation.z = -0.086;
			// release_pose.pose.position.x-=0.05;


			// // if(!executeTask("PLACE",transfer_pose_s))
			// auto retracted_pose = get_current_pose();
			// retracted_pose.pose.position.z+=0.1;
			// moveit_node.plan_and_execute_via_waypoints(retracted_pose);
			// utilities::writeToLog("REGRASP");
			// retracted_pose.pose.position = release_pose.pose.position;
			// moveit_node.plan_and_execute_via_waypoints(retracted_pose);
			// topple(source_bin, object_label);
			// utilities::writeToLog("Finish REGRASP");
		}


		
		// if(!moveit_node.plan_and_execute_via_waypoints(release_pose))
		// {
		// 	moveit_node.plan_and_execute(release_pose);
		// 	// executeTask("RELEASE");
		// 	ROS_ERROR_STREAM("FAILED TO RETURN TO A RETRACTED POSE.");
			
		// }

		auto intermediate_release = get_current_pose().pose;
		intermediate_release.position.z+=0.1;
		moveit_node.plan_and_execute_via_waypoints(intermediate_release);

		if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
		{
			moveit_node.plan_and_execute(transfer_pose_s);
			// executeTask("RELEASE");
			ROS_ERROR_STREAM("FAILED TO RETURN TO A SAFE POSE.");
			
		}
		executeTask("RELEASE");
	}
	utilities::writeToLog("Finish pick_and_drop "+std::to_string(success));


	if(success)
	{
		regrasp_failure = 0;
	}
	else
	{
		ROS_INFO_STREAM("PICK AND DROP FAILED. \n Enter y to increment regrasp failure counter ("<<regrasp_failure<<") ...\n NOTE: If it reaches 5 the process will terminate.");
		std::string input;
		std::cin>>input;
		if(input == "y")
		{
			regrasp_failure ++;
		}
		else
		{
			regrasp_failure  = 0;
		}
	}



	return success;
}


bool TaskPlanner::pick_and_drop_without_push(const geometry_msgs::PoseStamped& grasp_point,  geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP)
{
	// Overloaded function for PICK_AND_PLACE and PICK_AND_MOVE.
	ROS_INFO_STREAM(" PERFORMING A PICK AND DROP QUERY");
	ROS_WARN_STREAM("drop pose:"<<drop_pose.pose.position.x << ","<<drop_pose.pose.position.y<<","<<drop_pose.pose.position.z);
	bool success = false;
	bool ADJUSTED_DROP_SUCCESS = true;
	int OBJECT_ATTACHMENT = 0;
	bool IS_OBJECT_ATTACHED = true;

	PlacePlanClient ac("/placeplan_action", true);
	//ac.waitForServer();
	task_planning::PlacePlanGoal placeplan_goal;
    placeplan_goal.target_object_pose = target_obj_pose;
    placeplan_goal.target_object_name = "dove";
    //ac.sendGoal(placeplan_goal, &placeplan_doneCb, &placeplan_activeCb, &placeplan_feedbackCb);
    target_obj_pose.position.z += 0.06;
    pre_push_pose_ = target_obj_pose;
/*
	task_planning::PlacePlan srv;
	ros::ServiceClient placeplan_service_client = node_handle.serviceClient<task_planning::PlacePlan>("/placeplan_service");
	srv.request.target_object_pose = target_obj_pose;
	// srv.request.target_object_pose.position.x += X_CALIBRATION_OFFSET;
	// srv.request.target_object_pose.position.y += Y_CALIBRATION_OFFSET;

	srv.request.target_object_name = object_label;
	if (placeplan_service_client.call(srv))
	{
		pre_push_pose_  = srv.response.pre_push_pose;
		post_push_pose_ = srv.response.post_push_pose;
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
*/
    ros::spinOnce();

	ros::ServiceClient clt_pusher;
	auto transfer_pose_s = transfer_pose_1;
	auto transfer_pose_t = transfer_pose_2;
	if(source_bin == "bin_2" && target_bin == "bin_1")
	{
		transfer_pose_s = transfer_pose_2;
		transfer_pose_t = transfer_pose_1;
	}	
	//Toggle behavior of going to the center versus going over grid
	// transfer_pose_t.pose.position.z-=0.25;
	// double z_offset_before_drop = -0.15;
	
	// transfer_pose_t = drop_pose;

	// double z_offset_before_drop = -0.1;

	double z_offset_before_drop = -0.1;
	auto sensing_structures = comm->current_sensing_structures();
	utilities::writeToLog("Inside pick_and_drop");
	if(executeTask("PICK",grasp_point, size))
	{
		ROS_ERROR_STREAM("\n\n\n pick_and_drop PICKED");
		if(REGRASP)
		{
			success = false;
		}
		else
		{
			OBJECT_ATTACHMENT = is_object_attached(source_bin, object_label, grasped_obj_pose);
			IS_OBJECT_ATTACHED = OBJECT_ATTACHMENT == 1;
			if(IS_OBJECT_ATTACHED)
			{
				drop_pose.pose.position.z += 0.06;
				auto retracted_drop = drop_pose.pose;
				retracted_drop.position.z-=z_offset_before_drop;

				auto viewpoint_pose = transfer_pose_t.pose;
				viewpoint_pose.orientation = drop_pose.pose.orientation;
				viewpoint_pose.position.z=retracted_drop.position.z;

				// if(planToStartOfPreC("TRANSFER", source_bin, target_bin, viewpoint_pose, (int)PREGRASP_TO_DROP))
				if(planToStartOfPreC("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
				{
				
					// delayed_release(4.0);
					// if(executeTask("TRANSFER", source_bin, target_binp, viewpoint_pose, (int)PREGRASP_TO_DROP))
					if(executeTask("TRANSFER", source_bin, target_bin, retracted_drop, (int)PREGRASP_TO_DROP))
					{
						ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM SOURCE TO TARGET");
						//while(!is_placeplan_ready_);
						//Sense, offset, move, drop
						// std::string input;
						// std::cin>>input;
						utilities::writeToLog("Starting adjusted_drop");
						ADJUSTED_DROP_SUCCESS = adjusted_drop(target_obj_pose, pre_push_pose_,z_offset_before_drop, target_bin, object_label);
						utilities::writeToLog("adjusted_drop result:"+std::to_string(ADJUSTED_DROP_SUCCESS));
						// executeTask("RELEASE");

#ifdef USE_PUSHING
						moveit_node.addBin();
						// planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP);
						if(ADJUSTED_DROP_SUCCESS){
							bool safe_move_away = true;
							auto safe_move_pose = transfer_pose_t;
							sleep(SENSING_SLEEP);
							safe_move_pose.pose.orientation = get_current_pose().pose.orientation;
							safe_move_pose.pose.position.z = 0.46;

							safe_move_away = moveit_node.plan_and_execute_via_waypoints(safe_move_pose);
							if(!safe_move_away)
							{
								safe_move_away = moveit_node.plan_and_execute(safe_move_pose);
								// executeTask("RELEASE");
								ROS_ERROR_STREAM("FAILED TO RETURN OVERHEAD SAFE POSE.");
								
							}
							if(safe_move_away)
							{
									ros::NodeHandle nh;
									utilities::writeToLog("Calling Push Adjustment - inter");
									ROS_INFO_STREAM("Hi, Changkyu");
									ros::ServiceClient clt_pusher
								     = nh.serviceClient<rl_msgs::organizing_pusher_jdxdemo_srv>(
								       "/iiwa/changkyu/organizer_jdxdemo/pusher");
								    rl_msgs::organizing_pusher_jdxdemo_srv::Request req;
								    rl_msgs::organizing_pusher_jdxdemo_srv::Response res;
								    req.camera_name = "camera2";
								    req.param="thresh=0.01,iter_max=4";
								    clt_pusher.call(req, res);    
								    ROS_INFO_STREAM("Bye Changkyu");
								    utilities::writeToLog("Done with Push Adjustment - inter");						
							}
						}else{
							ROS_ERROR_STREAM("\n\n\n !!!pick_and_drop adjusted drop failed");
						}
						moveit_node.deleteBin();

#endif
						sleep(SENSING_SLEEP);
						ROS_ERROR_STREAM("ADJUSTED_DROP_SUCCESS:"<<ADJUSTED_DROP_SUCCESS);
						comm->set_sensing(std::get<0>(sensing_structures), std::get<1>(sensing_structures));
						sleep(SENSING_SLEEP);
						//ROS_ERROR_STREAM("Waiting for the input");
						// std::string xxx;
						// std::cin>>xxx;
						comm->start_sensing();
						if(planToStartOfPreC("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP) && executeTask("TRANSFER", target_bin, source_bin, transfer_pose_s.pose, (int)DROP_TO_PREGRASP))
						{
							ROS_ERROR_STREAM("\n\n\n pick_and_drop MOVED FROM TARGET BACK TO SOURCE");
							success = true;
						}
						else
						{
							utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");	
							ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION BACK");
						}
						if(!ADJUSTED_DROP_SUCCESS)
						{

							ROS_ERROR_STREAM("\n\n\nFailed to successfully complete ADJUSTED DROP");
							executeTask("RELEASE");
							return false;
						}
					}
					else
					{
						utilities::writeToLog("pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
						ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE PRECOMPUTATION FWD");
					}
				}
			}
			else
			{
				utilities::writeToLog("pick_and_drop OBJECT NOT ATTACHED");	
				ROS_ERROR_STREAM("\n\n\n pick_and_drop FAILED TO EXECUTE OBJECT NOT ATTACHED");
			}
		}
	}
	else
	{
		utilities::writeToLog("Failed to go to pick pose");
		ROS_ERROR_STREAM("\n\n\nFAILED TO PICK");
	}

	if(!success)
	{
		transfer_pose_s = transfer_pose_1;
		transfer_pose_t = transfer_pose_2;
		if(source_bin == "bin_2" && target_bin == "bin_1")
		{
			transfer_pose_s = transfer_pose_2;
			transfer_pose_t = transfer_pose_1;
		}

		auto release_pose = transfer_pose_s;
		release_pose.pose.position.z-=0.08;

		// delayed_release(2);

		bool OBJECT_GRASPED_FROM_WRONG_FACE = !IS_OBJECT_ATTACHED && OBJECT_ATTACHMENT == -1;
		bool REGRASP_REQUIRED_FROM_DETECTION = REGRASP;
		if( OBJECT_GRASPED_FROM_WRONG_FACE || REGRASP_REQUIRED_FROM_DETECTION )
		{
			// // auto current_pose = get_current_pose().pose;
			// // current_pose.pose.position.x = 0.39;
			// // current_pose.pose.position.y = 0.34;
			// // current_pose.pose.position.z = 0.55;
			// ROS_ERROR_STREAM("ATTEMPTING TO RETURN TO A RETRACTED POSE.");

			// // release_pose.pose.orientation.w = 0.087;
			// // release_pose.pose.orientation.x = 0.008;
			// // release_pose.pose.orientation.y = 0.992;
			// // release_pose.pose.orientation.z = 0.087;
			// release_pose.pose.orientation.w = -0.130;
			// release_pose.pose.orientation.x = -0.011;
			// release_pose.pose.orientation.y = -0.988;
			// release_pose.pose.orientation.z = -0.086;
			// release_pose.pose.position.x-=0.05;


			// // if(!executeTask("PLACE",transfer_pose_s))
			// auto retracted_pose = get_current_pose();
			// retracted_pose.pose.position.z+=0.1;
			// moveit_node.plan_and_execute_via_waypoints(retracted_pose);
			utilities::writeToLog("REGRASP");
			// retracted_pose.pose.position = release_pose.pose.position;
			// moveit_node.plan_and_execute_via_waypoints(retracted_pose);
			topple(source_bin, object_label);
			utilities::writeToLog("Finish REGRASP");
		}


		
		// if(!moveit_node.plan_and_execute_via_waypoints(release_pose))
		// {
		// 	moveit_node.plan_and_execute(release_pose);
		// 	// executeTask("RELEASE");
		// 	ROS_ERROR_STREAM("FAILED TO RETURN TO A RETRACTED POSE.");
			
		// }

		auto intermediate_release = get_current_pose().pose;
		intermediate_release.position.z+=0.1;
		moveit_node.plan_and_execute_via_waypoints(intermediate_release);

		if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
		{
			moveit_node.plan_and_execute(transfer_pose_s);
			// executeTask("RELEASE");
			ROS_ERROR_STREAM("FAILED TO RETURN TO A SAFE POSE.");
			
		}
		executeTask("RELEASE");
	}
	utilities::writeToLog("Finish pick_and_drop "+std::to_string(success));
	return success;
}



void TaskPlanner::topple(std::string bin, std::string object_label)
{

	//Fallback Maneuver
	auto transfer_pose_s = transfer_pose_1;
	auto transfer_pose_t = transfer_pose_2;
	int bin_id = 1;
	double target_alignment_angle = -30;
	double sensing_alignment_angle = -120;
	if(bin == "bin_2")
	{
		transfer_pose_s = transfer_pose_2;
		transfer_pose_t = transfer_pose_1;
		bin_id = 2;
		target_alignment_angle = 30;
		sensing_alignment_angle = 120;
	}

	auto release_pose = transfer_pose_s;
	// release_pose.pose.position.z-=0.08;

	ROS_ERROR_STREAM("ATTEMPTING TO RETURN TO A RETRACTED POSE.");

	// release_pose.pose.orientation.w = 0.087;
	// release_pose.pose.orientation.x = 0.008;
	// release_pose.pose.orientation.y = 0.992;
	// release_pose.pose.orientation.z = 0.087;
	release_pose.pose.orientation.w = -0.130;
	release_pose.pose.orientation.x = -0.011;
	release_pose.pose.orientation.y = -0.988;
	release_pose.pose.orientation.z = -0.086;
	release_pose.pose.position.x-=0.05;


	// if(!executeTask("PLACE",transfer_pose_s))
	auto retracted_pose = get_current_pose();
	retracted_pose.pose.position.z+=0.1;
	moveit_node.plan_and_execute_via_waypoints(retracted_pose);
	
	retracted_pose.pose.position = transfer_pose_s.pose.position;
	moveit_node.plan_and_execute_via_waypoints(retracted_pose);


	
	if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
	{
		moveit_node.plan_and_execute(transfer_pose_s);
		// executeTask("RELEASE");
		ROS_ERROR_STREAM("FAILED TO RETURN TO A RETRACTED POSE.");
		
	}



	auto current_pose = get_current_pose().pose;
	std::vector<double> dimensions = {0.15,0.15,0.1};
	auto bb_pose = current_pose;
	bb_pose.position.z-=0.40;
	bb_pose.position.z-=dimensions[2]/2;
	bb_pose.position.z+=0.02;
	//xyz, qwqxqyqz
	std::vector<double> bb_pose_vec = {bb_pose.position.x, bb_pose.position.y, bb_pose.position.z, 1, 0, 0, 0};
	comm->set_sensing(bin_id+4, {object_label}, dimensions, bb_pose_vec);
	sleep(SENSING_SLEEP);
	comm->start_sensing();
	sleep(SENSING_SLEEP);
	auto sensing_result = comm->get_sensing();

	geometry_msgs::Pose plane_detection;
	//Blindly trust the detection since we do not reason about the grasps
	if(!sensing_result.empty())
		// && sensing_result[0].end_effector_pose.z>-1000)
	{
		plane_detection.position.x = sensing_result[0].detection.x;
		plane_detection.position.y = sensing_result[0].detection.y;
		plane_detection.position.z = sensing_result[0].detection.z;
		plane_detection.orientation.x = sensing_result[0].detection.qx;
		plane_detection.orientation.y = sensing_result[0].detection.qy;
		plane_detection.orientation.z = sensing_result[0].detection.qz;
		plane_detection.orientation.w = sensing_result[0].detection.qw;
		ROS_ERROR_STREAM("Sensing Result: "<<sensing_result[0].print());
		ROS_WARN_STREAM("DETECTED CENTER OF PLANE TO TOPPLE OBJECT: "<<sensing_result[0].detection.x<<" "<<sensing_result[0].detection.y<<" "<<sensing_result[0].detection.z<<" "<<sensing_result[0].detection.qx<<" "<<sensing_result[0].detection.qy<<" "<<sensing_result[0].detection.qz<<" "<<sensing_result[0].detection.qw<<" ");

		// adjusted_pose = placement_mod->get_adjusted_ee_pose(object_label, target_bin, current_pose, plane_detection, target_obj_pose);
	}
	else
	{
		ROS_ERROR_STREAM("Nothing Detected!");
	}
	std::string dummy;

	std::cout<<"Should hhave Sensed the plane \n";
	// std::cin>>dummy
;
	sleep(SENSING_SLEEP);

	geometry_msgs::Pose current_object, adjusted_pose;

	std::pair< geometry_msgs::Pose, std::vector< std::vector<geometry_msgs::Pose> > > regrasp_maneuvers;

	current_pose = get_current_pose().pose;
	auto intermediate_pose = transfer_pose_s.pose;
	intermediate_pose.position.z-=0.1;
	moveit_node.plan_and_execute_via_waypoints(intermediate_pose);
		

	std::cout<<"Just below the transfer s \n";
	// std::cin>>dummy;



	sleep(SENSING_SLEEP);
	bool sense_success = false;
	current_object = sense_pose(sense_success, get_current_pose().pose, bin, object_label);

	
	if(sense_success)
	{
		adjusted_pose = placement_mod->get_adjusted_aligned_ee_pose(object_label, bin, get_current_pose().pose, current_object, target_alignment_angle+90);
		moveit_node.plan_and_execute_via_waypoints(adjusted_pose);
		
		current_object = sense_pose(sense_success, get_current_pose().pose, bin, object_label);
		
		sense_success = sense_success && !placement_mod->is_top_surface_graspable(object_label, current_object);
	}


	if(sense_success)
	{
		regrasp_maneuvers = placement_mod->get_adjusted_regrasp_pose(object_label, bin, get_current_pose().pose, current_object, plane_detection, target_alignment_angle);

		adjusted_pose = regrasp_maneuvers.first;
		intermediate_pose = get_current_pose().pose;
		intermediate_pose.orientation = adjusted_pose.orientation;
		moveit_node.plan_and_execute_via_waypoints(intermediate_pose);

		std::cout<<"First Adjustment \n";
		// std::cin>>dummy;



		sleep(SENSING_SLEEP);
		// current_object = sense_pose(sense_success, get_current_pose().pose, bin, object_label);
		// adjusted_pose = placement_mod->get_adjusted_regrasp_pose(object_label, bin, current_pose, current_object, plane_detection, target_alignment_angle);
		
		moveit_node.plan_and_execute_via_waypoints(adjusted_pose);


		std::cout<<"Second Adjustment \n";
		// std::cin>>dummy;
		auto current_strategy = regrasp_maneuvers.second[0];
		if(current_strategy.size()>1)
		{
			for(int i=0; i<current_strategy.size()-1; ++i)
			{
				moveit_node.plan_and_execute_via_waypoints(current_strategy[i]);
			}
		}
		executeTask("RELEASE");

		moveit_node.plan_and_execute_via_waypoints(current_strategy.back());

		current_pose = get_current_pose().pose;
		intermediate_pose = current_pose;
		intermediate_pose.position.z+=0.15;
		moveit_node.plan_and_execute_via_waypoints(intermediate_pose);


		std::cout<<"Retract out \n";
		// std::cin>>dummy;
	}
	else
	{
		executeTask("RELEASE");
	}

	// if(!moveit_node.plan_and_execute_via_waypoints(transfer_pose_s))
	// {

	// 	moveit_node.plan_and_execute(transfer_pose_s);
	// 	// executeTask("RELEASE");
	// 	ROS_ERROR_STREAM("FAILED TO RETURN TO A RETRACTED POSE.");
		
	// }


	std::cout<<"Done with stuff \n";
	// std::cin>>dummy;

	// std::cout<<"Should have moved to the topple pose \n"<<topple_pose.pose<<"\n";
	// std::cin>>dummy;











	// auto release_pose = transfer_pose_s;
	// release_pose.pose.position.z-=0.08;

	// ROS_ERROR_STREAM("ATTEMPTING TO RETURN TO A RETRACTED POSE.");

	// // release_pose.pose.orientation.w = 0.087;
	// // release_pose.pose.orientation.x = 0.008;
	// // release_pose.pose.orientation.y = 0.992;
	// // release_pose.pose.orientation.z = 0.087;
	// release_pose.pose.orientation.w = -0.130;
	// release_pose.pose.orientation.x = -0.011;
	// release_pose.pose.orientation.y = -0.988;
	// release_pose.pose.orientation.z = -0.086;
	// release_pose.pose.position.x-=0.05;


	// // if(!executeTask("PLACE",transfer_pose_s))
	// auto retracted_pose = get_current_pose();
	// retracted_pose.pose.position.z+=0.1;
	// moveit_node.plan_and_execute_via_waypoints(retracted_pose);
	
	// retracted_pose.pose.position = release_pose.pose.position;
	// moveit_node.plan_and_execute_via_waypoints(retracted_pose);


	
	// if(!moveit_node.plan_and_execute_via_waypoints(release_pose))
	// {
	// 	moveit_node.plan_and_execute(release_pose);
	// 	// executeTask("RELEASE");
	// 	ROS_ERROR_STREAM("FAILED TO RETURN TO A RETRACTED POSE.");
		
	// }

}


geometry_msgs::PoseStamped TaskPlanner::get_orthogonal_pose(geometry_msgs::PoseStamped input_pose)
{
	Eigen::Quaternionf q;
	q.w() = input_pose.pose.orientation.w;
	q.x() = input_pose.pose.orientation.x;
	q.y() = input_pose.pose.orientation.y;
	q.z() = input_pose.pose.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();

	Eigen::Quaternionf z90;
	z90.x() = 0;
	z90.y() = 0;
	z90.z() = 0.70717;
	z90.w() = 0.70717;

	Eigen::Matrix3f z90rot;
	z90rot = z90.toRotationMatrix();

	Eigen::Matrix3f rotated_orientation = rotMat*z90rot;
	Eigen::Quaternionf rotated_quaternion(rotated_orientation);

	geometry_msgs::PoseStamped rotated_pose;
	rotated_pose.pose.position.x = input_pose.pose.position.x;
	rotated_pose.pose.position.y = input_pose.pose.position.y;
	rotated_pose.pose.position.z = input_pose.pose.position.z;
	rotated_pose.pose.orientation.x = rotated_quaternion.x();
	rotated_pose.pose.orientation.y = rotated_quaternion.y();
	rotated_pose.pose.orientation.z = rotated_quaternion.z();
	rotated_pose.pose.orientation.w = rotated_quaternion.w();


	return rotated_pose;
}

bool TaskPlanner::executeTask(std::string query)
{
	// Overloaded function for HOME.
	ROS_INFO_STREAM("Executing a " << query << " query...");
	if (query == "HOME")
	{
		// utilities::goToHomePosition(my_iiwa);
		bool success = moveit_node.goto_home();
		utilities::sleepForMotion(my_iiwa,2.0);
		return success;
	}
	else if (query == "CLOSE_FINGERS")
	{
		#ifdef REFLEX
		//#### ROBOTIQ
		grasp_node.publish_command("90");
		#endif
		#ifndef REFLEX
		grasp_node.publish_command("c");
		#endif
		return true;
	}
	else if( query == "GRASP" )
	{
		grasp_node.publish_command("c");
		ros::Duration(0.1).sleep();
	}
	else if( query == "RELEASE" )
	{
		grasp_node.publish_command("o");
		// ros::Duration(2.5).sleep();
		// ros::Duration(0.1).sleep();
	}
	else if(query == "PREPLAN")
	{
		 moveit_node.pre_plan_grid();
		return true;
	}
}	




bool TaskPlanner::executeTask(std::string query, const geometry_msgs::PoseStamped& target_pose, double size)
{
	// Overloaded function for MOVE and PLACE.
	bool openFingers = false;
	bool success;
	ROS_INFO_STREAM("Executing a " << query << " query...");
	if (query == "MOVE" || query == "STEER"){
		openFingers = false;
	
	}
	else if (query == "PLACE")
	{ 	
		openFingers = true;
	}
	else if (query == "MOVE" || query == "PLACE")
	{
		success = moveit_node.plan_and_execute(target_pose);
	
	}
	else if (query == "STEER")
	{
		// geometry_msgs::PoseStamped current_pose;
		// while (!my_iiwa.getCartesianPose(current_pose)) {}
		// moveit_node.plan_and_execute_via_waypoints(current_pose.pose,target_pose.pose,.0);
		my_iiwa.setCartesianPose(target_pose);
		success = checkIfReached(target_pose);
	}
	else if (query == "PICK")
	{ 
		success = grasp_node.grasp(target_pose, size, moveit_node, my_iiwa);
		openFingers = false;
	}	

		
	
	if (openFingers) 
	{
		// gripper_command.data = "o";
		// gripper_command_publisher.publish(gripper_command);
		executeTask("RELEASE");
	}

	return success;
}

bool TaskPlanner::executeTask(std::string query, const std::vector<double>& target_pose)
{
	// Overloaded function for MOVE and PLACE.
	bool openFingers;
	ROS_INFO_STREAM("Executing a " << query << " query...");
	if (query == "MOVE")
		openFingers = false;
	else if (query == "PLACE")
		openFingers = true;

	bool success;

	if (query == "MOVE" || query == "PLACE")
		success = moveit_node.plan_and_execute(target_pose);
	

	//utilities::sleepForMotion(my_iiwa,2.0);

	
	
	if (openFingers) 
	{
		// gripper_command.data = "o";
		// gripper_command_publisher.publish(gripper_command);
		executeTask("RELEASE");
	}

	if(!success)
		ROS_ERROR_STREAM("Execute Task to joint configuration failed.");

	return success;
}

bool TaskPlanner::planToStartOfPreC(std::string query,  std::string source_bin, std::string target_bin, int mode)
{
	bool success = true;
	if(query == "TRANSFER"){
		ROS_INFO_STREAM("Executing a " << query << " query...");
		moveit_node.planToStartOfPreC(source_bin, target_bin, mode);
		
	}
	
	return success;
}


//mode
//0: from pre_grasp to drop(higher to lower)
//1: from drop to pre_grasp(lower to higher)
bool TaskPlanner::executeTask(std::string query,  std::string source_bin, std::string target_bin, int mode)
{
	bool success = true;
	if(query == "TRANSFER"){
		ROS_INFO_STREAM("Executing a " << query << " query...");
		moveit_node.execute(source_bin, target_bin, mode);
		
	}
	
	return success;
}



bool TaskPlanner::planToStartOfPreC(std::string query,  std::string source_bin, std::string target_bin, geometry_msgs::Pose target_pose, int mode)
{
	bool success = true;
	ROS_INFO_STREAM("PLACEMENT DROP : Executing a " << query << " query..."<< target_pose.position.x<<" "<<target_pose.position.y<<" "<<target_pose.position.z<<" ");
	if(query == "TRANSFER"){
		success = moveit_node.planToStartOfPreC(source_bin, target_bin, target_pose, mode);
		
	}
	
	return true;
}


//mode
//0: from pre_grasp to drop(higher to lower)
//1: from drop to pre_grasp(lower to higher)
bool TaskPlanner::executeTask(std::string query,  std::string source_bin, std::string target_bin,  geometry_msgs::Pose target_pose, int mode)
{
	bool success = true;
	ROS_INFO_STREAM("PLACEMENT DROP : Executing a " << query << " query..."<< target_pose.position.x<<" "<<target_pose.position.y<<" "<<target_pose.position.z<<" ");
	if(query == "TRANSFER"){
		success = moveit_node.execute(source_bin, target_bin, target_pose, mode);
		
	}

	if(success && mode == (int)PREGRASP_TO_DROP)
	{
		geometry_msgs::PoseStamped drop_pose;
		drop_pose.pose = target_pose;
		success = moveit_node.plan_and_execute_via_waypoints(drop_pose);
	}
	return success;
}

void TaskPlanner::addCollisionObject(std::string objId, std::vector<float> objDim, std::vector<float> objPose)
{
	
}

// void TaskPlanner::addCollisionObjectFromMesh(std::string meshFile, std::vector<float> objPose)
// {
// 	moveit_node.addCollisionObjectFromMesh(meshFile,objPose);
// }

bool TaskPlanner::checkIfReached(geometry_msgs::PoseStamped target_pose)
{
	geometry_msgs::PoseStamped current_pose = get_current_pose();
	if (abs(current_pose.pose.position.x - target_pose.pose.position.x) <= goal_tolerance &&
		abs(current_pose.pose.position.y - target_pose.pose.position.y) <= goal_tolerance &&
		abs(current_pose.pose.position.z - target_pose.pose.position.z) <= goal_tolerance)
		return true;
	return false;
}

geometry_msgs::PoseStamped TaskPlanner::get_current_pose()
{
	geometry_msgs::PoseStamped current_pose;
	while (!my_iiwa.getCartesianPose(current_pose)) {}
	return current_pose;
}

bool TaskPlanner::is_object_attached(std::string current_bin)
{
	return true;
	// auto current_pose = get_current_pose();
	// current_pose.pose.position.z -= 0.43;
	// ros::ServiceClient grasp_planning_client = node_handle.serviceClient<grasping::GraspSuccess>("/GraspSuccess");
	// grasping::GraspSuccess srv;
	// if(bin_id == "bin_2")
	// 	srv.request.binId = 1;
	// else if(bin_id == "bin_1")
	// 	srv.request.binId = 2;
	// srv.request.x = current_pose.pose.position.x;
	// srv.request.y = current_pose.pose.position.y;
	// srv.request.z = current_pose.pose.position.z;
	// int gripper_collision_num = 0;
	// if (grasp_planning_client.call(srv))
	// {
	// 	gripper_collision_num = srv.response.collision_num;
	// 	if(gripper_collision_num > GRASP_DETECTION_THRESHOLD)
	// 		return true;
	// 	return false;
	// }
	// else
	// {
	// 	ROS_ERROR_STREAM("Grasp Success Service could not be contacted....");
	// 	return false;
	// }
}



int TaskPlanner::is_object_attached(std::string current_bin, std::string object_label, geometry_msgs::Pose grasped_obj_pose, bool check_for_sensing)
{

	int bin_id;
	if(current_bin=="bin_1")
	{
		bin_id = 3;
	}
	else if(current_bin=="bin_2")
	{
		bin_id = 4;
	}


	auto overhead_bin_pose = transfer_pose_1;
	if(current_bin == "bin_2")
	{
		overhead_bin_pose = transfer_pose_2;
	}	

	auto sense_pose = overhead_bin_pose;
	double rot_angle = 10 * 3.1418 / 180;
	//Offset by 10degrees on the Y axis
	sense_pose.pose.orientation.w =  0.087;
	sense_pose.pose.orientation.x = 0;
	sense_pose.pose.orientation.y = 0.996;
	sense_pose.pose.orientation.z = 0;
	sense_pose.pose.position.z-=0.12;

	auto retracted_pose = sense_pose;
	geometry_msgs::Pose current_pose = get_current_pose().pose;
	retracted_pose.pose.orientation = current_pose.orientation;

	// if(!moveit_node.plan_and_execute_via_waypoints(retracted_pose)){
	// 	if(!moveit_node.plan_and_execute(retracted_pose)){
	// 		moveit_node.plan_and_execute(overhead_bin_pose);
	// 		moveit_node.plan_and_execute_via_waypoints(retracted_pose);
	// 	}
	// }
	// // sense_pose.pose.position.x-=0.05;
	// if(!moveit_node.plan_and_execute_via_waypoints(sense_pose))
	// {	
	// 	moveit_node.plan_and_execute(sense_pose);
	// 	ROS_ERROR_STREAM("FAILED TO RETURN TO A SENSE POSE.");
	// 	// moveit_node.plan_and_execute_via_waypoints(overhead_bin_pose);
	// }
	// else
	// {
	// 	ROS_ERROR_STREAM("SUCCEEDED IN RETURNING TO A SENSE POSE.");
	// }


	if(moveit_node.plan_and_execute_via_waypoints(retracted_pose))
	{	
		ROS_WARN_STREAM("SUCCEEDED IN RETURNING TO A SENSE POSE.");
		moveit_node.plan_and_execute_via_waypoints(sense_pose);
	}
	else
	{
		moveit_node.plan_and_execute(overhead_bin_pose);
		ROS_ERROR_STREAM("FAILED TO RETURN TO A SENSE POSE.");
	}

	std::vector<double> dimensions = {0.15,0.15,0.15};
	// std::vector<double> dimensions = {0.15,0.15,0.1};

	double stem_offset = 0.40 + (dimensions[2]/2) - 0.02;

	double offset_x = stem_offset*sin(rot_angle);
	double offset_z = stem_offset*cos(rot_angle);


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;
	auto bb_pose = current_pose;
	
	//Account for the offset caused by the angle
	bb_pose.position.z-=offset_z;
	bb_pose.position.x+=offset_x;

	// bb_pose.position.z-=dimensions[2]/2;
	geometry_msgs::Pose obj_detection;

	bool debug_bb = false;
	do
	{

		//xyz, qwqxqyqz
		std::vector<double> bb_pose_vec = {bb_pose.position.x, bb_pose.position.y, bb_pose.position.z, 1, 0, 0, 0};
		comm->set_sensing(bin_id, {object_label}, dimensions, bb_pose_vec);
		// sleep(5);
		sleep(SENSING_SLEEP);
		comm->start_sensing();
		sleep(SENSING_SLEEP);
		auto sensing_result = comm->get_sensing();

		//Blindly trust the detection since we do not reason about the grasps
		if(!sensing_result.empty())
			// && sensing_result[0].end_effector_pose.z>-1000)
		{
			obj_detection.position.x = sensing_result[0].detection.x;
			obj_detection.position.y = sensing_result[0].detection.y;
			obj_detection.position.z = sensing_result[0].detection.z;
			obj_detection.orientation.x = sensing_result[0].detection.qx;
			obj_detection.orientation.y = sensing_result[0].detection.qy;
			obj_detection.orientation.z = sensing_result[0].detection.qz;
			obj_detection.orientation.w = sensing_result[0].detection.qw;
			ROS_WARN_STREAM("DETECTED OBJECT IN BOUNDING BOX: "<<sensing_result[0].detection.x<<" "<<sensing_result[0].detection.y<<" "<<sensing_result[0].detection.z<<" "<<sensing_result[0].detection.qx<<" "<<sensing_result[0].detection.qy<<" "<<sensing_result[0].detection.qz<<" "<<sensing_result[0].detection.qw<<" ");


		}
		else
		{
			ROS_ERROR_STREAM("Nothing Detected!");
			return 0;
		}


		if(debug_bb)
		{
			std::string input;
			std::cout<<"boduning box check/// ";
			std::cin>>input;
		}

	}while(debug_bb);

	// Eigen::Quaternionf q;
	// q.w() = obj_detection.orientation.w;
	// q.x() = obj_detection.orientation.x;
	// q.y() = obj_detection.orientation.y;
	// q.z() = obj_detection.orientation.z;
	// Eigen::Matrix3f rotMat;
	// rotMat = q.toRotationMatrix();
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// //TODO:: MAKE THIS GENERAL
	// Eigen::Vector3f reference_direction(0,1,0);
	// Eigen::Vector3f top(0,0,1);
	// Eigen::Vector3f reference_direction_world = rotMat*reference_direction;

	// ///Should be exactly 0.7071 but keeping some slack
	// if (std::abs(reference_direction_world.dot(top))>0.6)
	// {
	// 	return 1;
	// }
	// else
	// {
	// 	return -1;
	// }






	auto initial_object_pose_world = grasped_obj_pose;
	auto initial_ee_orientation_world = overhead_bin_pose.pose.orientation;
	auto final_ee_orientation_world = sense_pose.pose.orientation;



	// geometry_msgs::Pose final_object_pose_world.orientation = ?;
	// Eigen::Quaternionf initial_object_pose_world_q;
	// initial_object_pose_world_q.x() = initial_object_pose_world.x;
	// initial_object_pose_world_q.y() = initial_object_pose_world.y;
	// initial_object_pose_world_q.z() = initial_object_pose_world.z;
	// initial_object_pose_world_q.w() = initial_object_pose_world.w;
	// Eigen::Matrix3f initial_object_pose_world_m = initial_object_pose_world_q.toRotationMatrix();

	// std::string input;
	// std::cout<<"Enter stuff...";
	// std::cin>>input;
	// ROS_ERROR_STREAM("1");

	Eigen::Matrix3f initial_object_pose_world_m = get_rotation_matrix_from_orientation(initial_object_pose_world.orientation);
	// ROS_ERROR_STREAM("2");
	
	//Flatten the initial pose
	auto initial_pose_ret = placement_mod->check_primary_axis(object_label, grasped_obj_pose, true);
	// ROS_ERROR_STREAM("3");

	auto surface = initial_pose_ret.second;
	auto normal = surface->surface_normal();
	// ROS_ERROR_STREAM("4");

	normal.normalize();
	int index = -1;
	for(int i=0; i<3; ++i)
	{
		if(std::abs(std::abs(normal[i]) - 1) < 0.001)
		{
			index = i;
			break;
		}
	}

	ROS_ERROR_STREAM("Initial matrix: \n"<<normal<<"\n index: "<< index<<" matrix: \n"<<initial_object_pose_world_m);

	initial_object_pose_world_m.col(0)[2] = 0;
	initial_object_pose_world_m.col(1)[2] = 0;
	initial_object_pose_world_m.col(2)[2] = 0;


	initial_object_pose_world_m.col(index)[0] = 0;
	initial_object_pose_world_m.col(index)[1] = 0;
	initial_object_pose_world_m.col(index)[2] = normal[index];


	initial_object_pose_world_m.col(0).normalize();
	initial_object_pose_world_m.col(1).normalize();
	initial_object_pose_world_m.col(2).normalize();

	ROS_ERROR_STREAM("After everything wrt \n"<<normal<<"\n  matrix: \n"<<initial_object_pose_world_m);


	// std::cout<<"Enter stuff again...";
	// std::cin>>input;
	//Flatten the initial pose







	Eigen::Matrix3f initial_ee_orientation_world_m = get_rotation_matrix_from_orientation(initial_ee_orientation_world);
	Eigen::Matrix3f final_ee_orientation_world_m = get_rotation_matrix_from_orientation(final_ee_orientation_world);
	Eigen::Matrix3f ee_offset = initial_ee_orientation_world_m.inverse().eval() * final_ee_orientation_world_m;

	Eigen::Matrix3f final_object_pose_world_m = (initial_object_pose_world_m.inverse().eval()*ee_offset).inverse().eval();



	// Eigen::Quaternionf final_object_pose_world_q ( initial_object_pose_world_m * ee_offset );
	Eigen::Quaternionf final_object_pose_world_q ( final_object_pose_world_m );

	auto before_rot = in_degrees(placement_mod->check_primary_axis(object_label, grasped_obj_pose));

	grasped_obj_pose.orientation = get_quaternion_from_eigen_quaternion(final_object_pose_world_q);
	auto after_rot = in_degrees(placement_mod->check_primary_axis(object_label, grasped_obj_pose));



	double grasped_axis = placement_mod->check_primary_axis(object_label, grasped_obj_pose);
	double sensed_axis = placement_mod->check_primary_axis(object_label, obj_detection);

	ROS_INFO_STREAM("---------------------------------");
	ROS_INFO_STREAM("---------------------------------");
	ROS_INFO_STREAM("---------------------------------");
	ROS_INFO_STREAM("---------------------------------");

	ROS_INFO_STREAM(initial_ee_orientation_world);
	ROS_INFO_STREAM("---------------------------------");
	ROS_INFO_STREAM(final_ee_orientation_world);

	ROS_WARN_STREAM("Primary Axis Alignment before rotation.....\n"<<initial_object_pose_world_m);
	ROS_ERROR_STREAM("Primary Axis Alignment before rotation.....\n"<<before_rot);
	ROS_WARN_STREAM("Primary Axis Alignment after rotation.....\n"<<final_object_pose_world_q.toRotationMatrix());
	ROS_ERROR_STREAM("Primary Axis Alignment after rotation.....\n"<<after_rot);
	ROS_WARN_STREAM("Sensed Axis Alignment after rotation.....\n"<<get_rotation_matrix_from_orientation(obj_detection.orientation));
	ROS_ERROR_STREAM("Sensed Axis Alignment after rotation.....\n"<<in_degrees(sensed_axis));
	

	// std::string input;
	// std::cin>>input;


	double pi = 3.1418;

	// if(grasped_axis<0)
	// 	grasped_axis=(2*pi)+grasped_axis;
	// if(sensed_axis<0)
	// 	sensed_axis=(2*pi)+sensed_axis;


	// if(grasped_axis>pi)
	// 	grasped_axis-=pi;
	// if(sensed_axis>pi)
	// 	sensed_axis-=pi;

	auto symmetry_angle = [](double angle) 
	{  
		double pi = 3.1418;
		if(angle<0)
		{
			angle = (2*pi)+angle;
		}
		if(angle>pi)
		{
			angle -= pi;
		}
		if(angle > pi/2)
		{
			angle -= pi;
		}
		return angle;
	};


	double symmetry_grasped = symmetry_angle(grasped_axis);
	double symmetry_sensed = symmetry_angle(sensed_axis);

	



	bool sensing_certainty = true;
	// if( std::abs(grasped_axis - sensed_axis) > pi/4 )
	if( std::abs(sensed_axis -  grasped_axis) > pi/4 )
	{
		sensing_certainty = false;
		utilities::writeToLog("Sensing uncertainty because alignment "+std::to_string(grasped_axis)+" vs "+std::to_string(sensed_axis)+" did not match.");
	}
	if(check_for_sensing && !sensing_certainty)
	{
		return 0;
	}




	bool graspable = placement_mod->is_top_surface_graspable(object_label, obj_detection);

	if(graspable)
	{
		return 1;
	}
	else
	{
		return -1;
	}


}

void TaskPlanner::validateCalibration(){
	geometry_msgs::PoseStamped pre_pose_1, pre_pose_2, pre_pose_3, pre_pose_4;
	pre_pose_1.pose.position.x = 0.45;
	pre_pose_1.pose.position.y = -0.32;
	pre_pose_1.pose.position.z = 0.2;
	//pre_pose_1.pose.orientation.x = -0.38268343;
	//pre_pose_1.pose.orientation.y = 0.92387953;
	pre_pose_1.pose.orientation.x = 0;
	pre_pose_1.pose.orientation.y = 1;
	
	pre_pose_1.pose.orientation.z = 0.0;
	pre_pose_1.pose.orientation.w = 0.0;
	//moveit_node.plan_and_execute(pre_pose_1);
	//moveit_node.test_grid_move();
	
	moveit_node.move_grid(true);
	while(1);
}


bool TaskPlanner::adjusted_drop_with_tightenv(geometry_msgs::Pose target_obj_pose, std::vector<geometry_msgs::Pose>  pre_push_pose_list,double z_offset, std::string target_bin, std::string object_label)
{
	bool success = true;
	sleep(SENSING_SLEEP);
	auto ps = get_current_pose();
	auto current_pose = ps.pose;
	bool adjustment_success;


	double sense_z;
	double raised_z;
	double pose_z;

	ROS_WARN_STREAM("target_obj_pose:"<<target_obj_pose);

	// Eigen::Vector2f offsets = {target_obj_pose.position.x - pre_push_pose.position.x,  target_obj_pose.position.y - pre_push_pose.position.y};
	geometry_msgs::Pose pre_push_pose_raised = pre_push_pose_list[0];
	geometry_msgs::Pose pre_push_pose_lowered = pre_push_pose_list[1];
	

	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;
	geometry_msgs::Pose adjusted_pose;





	adjusted_pose = adjust_pose(adjustment_success, current_pose, pre_push_pose_raised, target_bin, object_label);
	if(!adjustment_success)
	{
		utilities::writeToLog("adjust_pose: pose flipped at target_bin");
		return false;
	}

	sense_z = current_object_detection.position.z;
	raised_z = pre_push_pose_list[0].position.z;
	pose_z = pre_push_pose_list[1].position.z;

	current_pose.position.z += (raised_z-sense_z);
	success = moveit_node.plan_and_execute_via_waypoints(current_pose);

	//Lower it to be same z as raised_z
	adjusted_pose.position.z += (raised_z-sense_z);
	success = moveit_node.plan_and_execute_via_waypoints(adjusted_pose);


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;
	double current_z = current_pose.position.z;



	double delx = target_obj_pose.position.x - pre_push_pose_raised.position.x;
	double dely = target_obj_pose.position.y - pre_push_pose_raised.position.y;

	//At the same height push to the target object pose
	adjusted_pose.position.x = current_pose.position.x + delx;
	adjusted_pose.position.y = current_pose.position.y + dely;
	adjusted_pose.position.z = current_z;
	success = moveit_node.plan_and_execute_via_waypoints(adjusted_pose);

	
	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;
	current_z = current_pose.position.z;

	delx = pre_push_pose_lowered.position.x - target_obj_pose.position.x;
	dely = pre_push_pose_lowered.position.y - target_obj_pose.position.y;

	//lower it to the prepush pose lowered at the height of the object
	adjusted_pose.position.x = current_pose.position.x + delx;
	adjusted_pose.position.y = current_pose.position.y + dely;
	// adjusted_pose.position.z = current_z + (pose_z-raised_z);

	success = moveit_node.plan_and_execute_via_waypoints(adjusted_pose);


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;
	current_z = current_pose.position.z;
	adjusted_pose = current_pose;
	adjusted_pose.position.z = current_z + (pose_z-raised_z);
	success = moveit_node.plan_and_execute_via_waypoints(adjusted_pose);


	
	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;
	current_z = current_pose.position.z;


	delx = target_obj_pose.position.x - pre_push_pose_lowered.position.x;
	dely = target_obj_pose.position.y - pre_push_pose_lowered.position.y;

	//lower it to the prepush pose lowered at the height of the object
	adjusted_pose.position.x = current_pose.position.x + delx;
	adjusted_pose.position.y = current_pose.position.y + dely;
	adjusted_pose.position.z = current_z;

	success = moveit_node.plan_and_execute_via_waypoints(adjusted_pose);




	success = success && executeTask("RELEASE");


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose; //=target_ee_pose
	adjusted_pose = current_pose;
	adjusted_pose.position.z -= z_offset;

	success = success && moveit_node.plan_and_execute_via_waypoints(adjusted_pose);
	

	return success;
}

bool TaskPlanner::adjusted_drop(geometry_msgs::Pose target_obj_pose, geometry_msgs::Pose  pre_push_pose,double z_offset, std::string target_bin, std::string object_label)
{
	bool success = true;
	sleep(SENSING_SLEEP);
	auto ps = get_current_pose();
	auto current_pose = ps.pose;
	bool adjustment_success;
	// ROS_WARN_STREAM("target_obj_pose:"<<target_obj_pose.position.x<<","<<target_obj_pose.position.y<<","<<target_obj_pose.position.z<<",");
	ROS_WARN_STREAM("target_obj_pose:"<<target_obj_pose);
	// auto adjusted_pose = adjust_pose(adjustment_success, current_pose, target_obj_pose, target_bin, object_label);

	Eigen::Vector2f offsets = {target_obj_pose.position.x - pre_push_pose.position.x,  target_obj_pose.position.y - pre_push_pose.position.y};

	auto z3_pre_push_pose = pre_push_pose;
	
	bool pre_push_computation_failure = (  std::abs (pre_push_pose.position.z  + 1000) < 0.001 );

	if(pre_push_computation_failure)
	{
		z3_pre_push_pose = target_obj_pose;
	}

	//Raise it
	z_offset+=SENSING_Z_ADJUSTMENT;
	z3_pre_push_pose.position.z -= z_offset;


	auto z1_pre_push_pose = pre_push_pose;
	auto z1_grid_pose = target_obj_pose;



	//##################################################
	//################MOVE TOWARDS CENTER###############
	//##################################################
	//##################################################


	auto overhead_bin_ps = transfer_pose_1;
	if(target_bin == "bin_2")
	{
		overhead_bin_ps = transfer_pose_2;
	}	
	// auto adjusted_pose = current_pose;
	auto overhead_bin_pose = overhead_bin_ps.pose;

	auto tighter_push_pose = current_pose;
	// double L1_disp = 0.03;
	//tighter_push_pose.position.z -= z_offset;
	double cx = overhead_bin_pose.position.x;
	double cy = overhead_bin_pose.position.y;

	double tx = tighter_push_pose.position.x;
	double ty = tighter_push_pose.position.y;

	double disp = sqrt( (tx-cx)*(tx-cx) +  (ty-cy)*(ty-cy) );
	double L1_disp = std::min(SENSING_OFFSET_TO_CENTER, disp);
	// double L1_disp = std::min(0.1, disp);

	double L1_disp_x = tx + (L1_disp/(disp+0.00001))*(cx-tx);
	double L1_disp_y = ty + (L1_disp/(disp+0.00001))*(cy-ty);

	tighter_push_pose.position.x = L1_disp_x;
	tighter_push_pose.position.y = L1_disp_y;

	success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);

	ROS_WARN_STREAM("TOWARDS CENTER: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);





	//##################################################
	//##################################################
	//##################################################



	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;
	geometry_msgs::Pose adjusted_pose;
	// while(1){  
	// 	adjusted_pose = adjust_pose(adjustment_success, current_pose, z3_pre_push_pose, target_bin, object_label);
	// 	std::string input;
	// 	std::cin>>input;
	// 	if(input == "q"){
	// 		break;
	// 	}
	// }
	adjusted_pose = adjust_pose(adjustment_success, current_pose, z3_pre_push_pose, target_bin, object_label);
	if(!adjustment_success)
	{
		utilities::writeToLog("adjust_pose: pose flapped at target_bin");
		return false;
	}

	//Move to Z3 pre push
	ROS_ERROR_STREAM("Move to Z3 pre push\n"<<adjusted_pose);
	adjusted_pose.position.z-=SENSING_Z_ADJUSTMENT;
	current_pose.position.z = adjusted_pose.position.z;
	success = moveit_node.plan_and_execute_via_waypoints(current_pose);
	adjusted_pose.position.y -= 0.005;
	adjusted_pose.position.x += 0.002;

	success = moveit_node.plan_and_execute_via_waypoints(adjusted_pose);


	if(pre_push_computation_failure)
	{
		utilities::writeToLog("PlacePlanAction failed ");
		executeTask("RELEASE");
		return true;
	}


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;

	auto target_ee_pose = current_pose;
	target_ee_pose.position.z += z_offset;

	//Move to Z1 pre push
	ROS_ERROR_STREAM("Move to Z1 pre push\n"<<target_ee_pose);
	success = success && moveit_node.plan_and_execute_via_waypoints(target_ee_pose);

	
	
	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose; //=target_ee_pose
	target_ee_pose = current_pose;
	target_ee_pose.position.x+=offsets[0];
	target_ee_pose.position.y+=offsets[1];
	// target_ee_pose.position.x=target_obj_pose.position.x;
	// target_ee_pose.position.y=target_obj_pose.position.y;


	Eigen::Vector2f extra_push;

	if( offsets.norm() > 0.001 )
	{
		// extra_push = offsets.normalized()*0.008;
		extra_push = offsets.normalized()*0.008;
	}
	else
	{
		extra_push[0] = 0;
		extra_push[1] = 0;
	}
	target_ee_pose.position.x+=extra_push[0];
	target_ee_pose.position.y+=extra_push[1];



	//Move to Z1 grid position
	ROS_ERROR_STREAM("Move to Z1 grid position\n"<<target_ee_pose);
	success = success && moveit_node.plan_and_execute_via_waypoints(target_ee_pose);

	success = success && executeTask("RELEASE");


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose; //=target_ee_pose
	target_ee_pose.position.z -= z_offset;

	//Move to Z3 grid position
	ROS_ERROR_STREAM("Move to Z3 grid position\n"<<target_ee_pose);
	success = success && moveit_node.plan_and_execute_via_waypoints(target_ee_pose);



	

	return success;
}

bool TaskPlanner::adjusted_drop_without_push(geometry_msgs::Pose target_obj_pose, geometry_msgs::Pose  pre_push_pose,double z_offset, std::string target_bin, std::string object_label)
{
	bool success = true;
	sleep(SENSING_SLEEP);
	auto ps = get_current_pose();
	auto current_pose = ps.pose;
	bool adjustment_success;
	// ROS_WARN_STREAM("target_obj_pose:"<<target_obj_pose.position.x<<","<<target_obj_pose.position.y<<","<<target_obj_pose.position.z<<",");
	ROS_WARN_STREAM("target_obj_pose:"<<target_obj_pose);
	// auto adjusted_pose = adjust_pose(adjustment_success, current_pose, target_obj_pose, target_bin, object_label);

	Eigen::Vector2f offsets = {target_obj_pose.position.x - pre_push_pose.position.x,  target_obj_pose.position.y - pre_push_pose.position.y};

	auto z3_pre_push_pose = pre_push_pose;
	
	bool pre_push_computation_failure = true;

	if(pre_push_computation_failure)
	{
		z3_pre_push_pose = target_obj_pose;
	}

	//Raise it
	z3_pre_push_pose.position.z -= z_offset;


	auto z1_pre_push_pose = pre_push_pose;
	auto z1_grid_pose = target_obj_pose;



	//##################################################
	//################MOVE TOWARDS CENTER###############
	//##################################################
	//##################################################


	auto overhead_bin_ps = transfer_pose_1;
	if(target_bin == "bin_2")
	{
		overhead_bin_ps = transfer_pose_2;
	}	
	// auto adjusted_pose = current_pose;
	auto overhead_bin_pose = overhead_bin_ps.pose;

	auto tighter_push_pose = current_pose;
	// double L1_disp = 0.03;
	//tighter_push_pose.position.z -= z_offset;
	double cx = overhead_bin_pose.position.x;
	double cy = overhead_bin_pose.position.y;

	double tx = tighter_push_pose.position.x;
	double ty = tighter_push_pose.position.y;

	double disp = sqrt( (tx-cx)*(tx-cx) +  (ty-cy)*(ty-cy) );
	double L1_disp = std::min(0.05, disp);

	double L1_disp_x = tx + (L1_disp/(disp+0.00001))*(cx-tx);
	double L1_disp_y = ty + (L1_disp/(disp+0.00001))*(cy-ty);

	tighter_push_pose.position.x = L1_disp_x;
	tighter_push_pose.position.y = L1_disp_y;

	success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);

	ROS_WARN_STREAM("TOWARDS CENTER: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);





	//##################################################
	//##################################################
	//##################################################



	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;


	auto adjusted_pose = adjust_pose(adjustment_success, current_pose, z3_pre_push_pose, target_bin, object_label);
	if(!adjustment_success)
	{
		utilities::writeToLog("adjust_pose: pose flapped at target_bin");
		return false;
	}

	//Move to Z3 pre push
	success = moveit_node.plan_and_execute_via_waypoints(adjusted_pose);

	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;

	auto target_ee_pose = current_pose;
	target_ee_pose.position.z += -0.4;
	if(pre_push_computation_failure)
	{
		utilities::writeToLog("PlacePlanAction failed ");
		executeTask("RELEASE");
		return true;
	}


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose;

	target_ee_pose = current_pose;
	target_ee_pose.position.z += z_offset;

	//Move to Z1 pre push
	success = success && moveit_node.plan_and_execute_via_waypoints(target_ee_pose);

	
	
	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose; //=target_ee_pose
	target_ee_pose = current_pose;
	target_ee_pose.position.x+=offsets[0];
	target_ee_pose.position.y+=offsets[1];
	// target_ee_pose.position.x=target_obj_pose.position.x;
	// target_ee_pose.position.y=target_obj_pose.position.y;

	Eigen::Vector2f extra_push = offsets.normalized()*0.008;
	target_ee_pose.position.x+=extra_push[0];
	target_ee_pose.position.y+=extra_push[1];



	//Move to Z1 grid position
	success = success && moveit_node.plan_and_execute_via_waypoints(target_ee_pose);

	success = success && executeTask("RELEASE");


	sleep(SENSING_SLEEP);
	current_pose = get_current_pose().pose; //=target_ee_pose
	target_ee_pose.position.z -= z_offset;

	//Move to Z3 grid position
	success = success && moveit_node.plan_and_execute_via_waypoints(target_ee_pose);




	//Z3_grid_pose
	//









	// if(!adjustment_success)
	// {
	// 	ROS_ERROR_STREAM("Could not adjust the pose because sensing returned nothing.");
	// 	return false;
	// }

	// // for robustness, always try to retract the biggest distance possible
	// geometry_msgs::Pose before_push_pose; 
	
	// before_push_pose = adjusted_pose;

	// //Verify Sensing
	// if(success = moveit_node.plan_and_execute_via_waypoints(before_push_pose))
	// {
	// 	// sleep(1);
	// 	current_pose = get_current_pose().pose;
	// 	adjusted_pose = adjust_pose(adjustment_success, current_pose, target_obj_pose, target_bin, object_label);
	// 	ROS_ERROR_STREAM("Verified Sensing... this should at least work.");
	// }
	// else
	// {
	// 	ROS_ERROR_STREAM("Initial Adjustment failed... Hope for the best and continue...");
	// 	return false;
	// }
	// std::string input;
	// auto pre_push_pose_higher = adjusted_pose;
	// //pre_push_pose_higher.position.z += 0.15;
	// pre_push_pose_higher.position.x = pre_push_pose.position.x;
	// pre_push_pose_higher.position.y = pre_push_pose.position.y;
	// ROS_WARN_STREAM("Moved to pre_push_pose higher: "<<pre_push_pose_higher);

	// moveit_node.plan_and_execute_via_waypoints(pre_push_pose_higher);
	// pre_push_pose_higher.position.z -= 0.1;	
	// ROS_WARN_STREAM("Moved to pre_push_pose lower");
	// std::cin>>input;
	// moveit_node.plan_and_execute_via_waypoints(pre_push_pose_higher);	
	// std::cin>>input;
	// adjusted_pose.position.z -= 0.1;
	// ROS_WARN_STREAM("Moved to target pose");

	// moveit_node.plan_and_execute_via_waypoints(adjusted_pose);	
	// std::cin>>input;

	//####################################################################################
	//####################################################################################
	//##################################TIGHTER PACKING###################################
	//####################################################################################
	//####################################################################################

	/*

	//center <- center of bin at current z3
	//Move towards center of bin
	//Lower above object but below bin
	//Move to adjusted pose at z2
	//Move to -offset from adjusted pose at z2
	//Move to -offset from adjusted pose at z1, the level of objects
	//Move to +offset from adjusted pose at z1
	//Release and retract
	ROS_WARN("after first adjustment");
	std::string input;
	std::cin>>input;

	auto overhead_bin_ps = transfer_pose_1;
	if(target_bin == "bin_2")
	{
		overhead_bin_ps = transfer_pose_2;
	}	
	// auto adjusted_pose = current_pose;
	auto overhead_bin_pose = overhead_bin_ps.pose;

	auto tighter_push_pose = adjusted_pose;
	// double L1_disp = 0.03;
	//tighter_push_pose.position.z -= z_offset;
	double cx = overhead_bin_pose.position.x;
	double cy = overhead_bin_pose.position.y;

	double tx = tighter_push_pose.position.x;
	double ty = tighter_push_pose.position.y;

	double disp = sqrt( (tx-cx)*(tx-cx) +  (ty-cy)*(ty-cy) );
	double L1_disp = std::min(0.05, disp);

	double L1_disp_x = tx + (L1_disp/(disp+0.00001))*(cx-tx);
	double L1_disp_y = ty + (L1_disp/(disp+0.00001))*(cy-ty);

	tighter_push_pose.position.x = L1_disp_x;
	tighter_push_pose.position.y = L1_disp_y;

	std::string str_in;
	double adaptable_offset = 0;

	while(!(success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose))){
		L1_disp += 0.005;
		L1_disp_x = tx + (L1_disp/(disp+0.00001))*(cx-tx);
		L1_disp_y = ty + (L1_disp/(disp+0.00001))*(cy-ty);
		tighter_push_pose.position.x = L1_disp_x;
		tighter_push_pose.position.y = L1_disp_y;
		if(L1_disp > 0.1){
			return false;
		}
	}
	ROS_WARN_STREAM("TOWARDS CENTER: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;
	sleep(SENSING_SLEEP);
	adjusted_pose = adjust_pose(adjustment_success, get_current_pose().pose, target_obj_pose, target_bin, object_label);
	if(!adjustment_success)
	{
		ROS_ERROR_STREAM("Could not adjust the pose the first time... because sensing returned nothing.");
		return false;
	}

	
	//Only rotate the end effector to match the adjusted pose here
	//move to rotated pose with same position
	//Sense again
	//Update adjusted pose
	tighter_push_pose = get_current_pose().pose;
	tighter_push_pose.orientation =  adjusted_pose.orientation;
	moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);	
	sleep(SENSING_SLEEP);
	adjusted_pose = adjust_pose(adjustment_success, get_current_pose().pose, target_obj_pose, target_bin, object_label);
	if(!adjustment_success)
	{
		ROS_ERROR_STREAM("Could not adjust the pose the second time... because sensing returned nothing.");
		return false;
	}
	tighter_push_pose.orientation =  adjusted_pose.orientation;
	moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);	


	tighter_push_pose.position.z+=(z_offset/2);
	success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);
	ROS_ERROR_STREAM("Z2: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;

	tighter_push_pose.position.x = adjusted_pose.position.x;
	tighter_push_pose.position.y = adjusted_pose.position.y;
	adaptable_offset = 0;
	while(!(success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose))){
		adaptable_offset += 0.005;
		tighter_push_pose.position.x += (adaptable_offset/(disp+0.00001))*(cx-tx);
		tighter_push_pose.position.y += (adaptable_offset/(disp+0.00001))*(cy-ty);
		if(adaptable_offset> 0.3){
			ROS_ERROR_STREAM("Failed to move to adjusted_pose after retract towards center");
			return false;
		}
	}
	ROS_WARN_STREAM("Z2 AJUSTED POSE: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;

	//std::pair<double, double> offset =  std::make_pair(0.017, -0.021);
	// std::vector<std::pair<double, double>> retract_vector_list = {std::make_pair(0.006, -0.042), std::make_pair(0.004, -0.028)
	//  ,std::make_pair(0.017, -0.021),std::make_pair(0.001, -0.007) };
	// std::vector<std::pair<double, double>> retract_vector_list = {std::make_pair(0.017, -0.032)};
	std::vector<std::pair<double, double>> retract_vector_list = {std::make_pair(0.01, -0.028)};
	
	 bool negative_offset_success = false;
	for(int i = 0; i < retract_vector_list.size() && !negative_offset_success; i++)
	{
		tighter_push_pose.position.x = adjusted_pose.position.x;
		tighter_push_pose.position.y = adjusted_pose.position.y;
		tighter_push_pose.position.x += retract_vector_list[i].first;
		tighter_push_pose.position.y += retract_vector_list[i].second;
		negative_offset_success = moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);
	}
	success = success && negative_offset_success;
	ROS_ERROR_STREAM("Z2 -OFFSET: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;

	
	tighter_push_pose.position.z+=(z_offset/2);
	success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);
	ROS_WARN_STREAM("Z1 -OFFSET: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;


	tighter_push_pose.position.x = adjusted_pose.position.x;
	tighter_push_pose.position.y = adjusted_pose.position.y;
	success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);
	ROS_ERROR_STREAM("Z1 AJUSTED POSE: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;

	tighter_push_pose.position.x -= 0.005;
	tighter_push_pose.position.y += 0.008;
	// success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);
	moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);
	ROS_WARN_STREAM("Z1 + PUSH: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;

	if(success)
		executeTask("RELEASE");
	tighter_push_pose.position.z-=z_offset;
	success = success && moveit_node.plan_and_execute_via_waypoints(tighter_push_pose);
	ROS_ERROR_STREAM("RETRACT: Just got done with moving to \n"<<tighter_push_pose<<" with success "<<success);
	// std::cin>>str_in;

	// success = true;
	*/

	//####################################################################################
	//####################################################################################
	//##################################TIGHTER PACKING###################################
	//####################################################################################
	//####################################################################################


	// //#############################PREHENSILE PRIMITIVE###############################
	// std::pair<double, double> retracted_vector;
	// std::vector<std::pair<double, double>> retract_vector_list = {std::make_pair(0.006, -0.042), std::make_pair(0.004, -0.028)
	// 	,std::make_pair(0.001, -0.007) };
	// for(int i = 0; i < retract_vector_list.size(); i++)
	// {
	// 	before_push_pose = adjusted_pose;
	// 	before_push_pose.position.x += retract_vector_list[i].first;
	// 	before_push_pose.position.y += retract_vector_list[i].second;
	// 	//---Offset, adjusted, raised
	// 	ROS_WARN_STREAM("Planning to pose: "<<print_pose(before_push_pose));
	// 	if(success = moveit_node.plan_and_execute_via_waypoints(before_push_pose))
	// 	{
	// 		ROS_WARN_STREAM("+++++++++++++++++++++++\nSuccess 1: (---Offset, adjusted, raised) Moved to top of before_push_pose use "<<i<<" retract vector");
	// 		before_push_pose.position.z += z_offset;			
	// 		//---Offset, adjusted, lowered
	// 		ROS_WARN_STREAM("Planning to pose: "<<print_pose(before_push_pose));
	// 		if(success = moveit_node.plan_and_execute_via_waypoints(before_push_pose))
	// 		{
	// 			ROS_WARN_STREAM("+++++++++++++++++++++++\nSuccess 2: (---Offset, adjusted, lowered) Moved to before_push_pose use "<<i<<" retract vector");
	// 			//Save the offset performed
	// 			retracted_vector = retract_vector_list[i];
	// 			break;
	// 		}
	// 		else
	// 		{
	// 			ROS_ERROR_STREAM("---------------------------\nFailure 2: (---Offset, adjusted, lowered) Moved to before_push_pose use "<<i<<" retract vector");
	// 			continue;
	// 		}
	// 	}else
	// 	{
	// 		ROS_ERROR_STREAM("---------------------------\nFailure 1: (---Offset, adjusted, raised) Moved to top of before_push_pose use "<<i<<" retract vector");
	// 		continue;
	// 	}
	// }

	// if(!success)
	// {
	// 	ROS_ERROR_STREAM("ERROR: No available retract vector");
	// 	// executeTask("RELEASE");
	// 	return false;
	// }
	

	// success = false;
	// auto final_push_pose = adjusted_pose;
	// // I updated the placement module to be lower. -Rahul
	// final_push_pose.position.z += (z_offset-0.005);
	// // final_push_pose.position.z += (z_offset);
	// //We need to lower first before pushing

	// // We should retract by the same amount we displaced
	// //final_push_pose.position.x -= 0.005;
	// //final_push_pose.position.y += 0.005;
	
	// //final_push_pose.position.x -= (retracted_vector.first + 0.005);
	// //final_push_pose.position.y -= (retracted_vector.second + 0.005);
	
	// while(!success)
	// { 
	// 	//---Not offset, adjusted, lowered
	// 	ROS_WARN_STREAM("Planning to pose: "<<print_pose(final_push_pose));
	// 	if(success = moveit_node.plan_and_execute_via_waypoints(final_push_pose))
	// 	{
	// 		executeTask("RELEASE");
	// 		//We might want to naively raise and directly go back to the pre-comp start after this -Rahul

	// 		//---Not offset, adjusted, raised
	// 		ROS_WARN_STREAM("+++++++++++++++++++++++\nSuccess 3: (---Not offset, adjusted, lowered) ");
	// 		ROS_WARN_STREAM("Planning to pose: "<<print_pose(adjusted_pose));
	// 		moveit_node.plan_and_execute_via_waypoints(adjusted_pose);
	// 		break;
	// 	}
	// 	else
	// 	{
	// 		ROS_WARN_STREAM("-----------------------\nFailed 3: (---Not offset, adjusted, lowered) ");
	// 		final_push_pose.position.z += 0.005;
	// 		final_push_pose.position.x += 0.001;
	// 		final_push_pose.position.y += -0.007;
	// 	}
	// }
	// //#############################PREHENSILE PRIMITIVE###############################
	

	return success;
}


geometry_msgs::Pose TaskPlanner::sense_pose(bool& success, geometry_msgs::Pose current_pose, std::string target_bin, std::string object_label)
{
	success = false;
	geometry_msgs::Pose adjusted_pose = current_pose;

	int bin_id;
	if(target_bin=="bin_1")
	{
		bin_id = 3;
	}
	else if(target_bin=="bin_2")
	{
		bin_id = 4;
	}

	std::vector<double> dimensions = {0.15,0.15,0.1};
	auto bb_pose = current_pose;
	bb_pose.position.z-=0.40;
	bb_pose.position.z-=dimensions[2]/2;
	bb_pose.position.z+=0.02;
	//xyz, qwqxqyqz
	std::vector<double> bb_pose_vec = {bb_pose.position.x, bb_pose.position.y, bb_pose.position.z, 1, 0, 0, 0};
	comm->set_sensing(bin_id, {object_label}, dimensions, bb_pose_vec);
	sleep(SENSING_SLEEP);
	comm->start_sensing();
	sleep(SENSING_SLEEP);
	auto sensing_result = comm->get_sensing();

	geometry_msgs::Pose obj_detection;
	//Blindly trust the detection since we do not reason about the grasps
	if(!sensing_result.empty())
		// && sensing_result[0].end_effector_pose.z>-1000)
	{
		obj_detection.position.x = sensing_result[0].detection.x;
		obj_detection.position.y = sensing_result[0].detection.y;
		obj_detection.position.z = sensing_result[0].detection.z;
		obj_detection.orientation.x = sensing_result[0].detection.qx;
		obj_detection.orientation.y = sensing_result[0].detection.qy;
		obj_detection.orientation.z = sensing_result[0].detection.qz;
		obj_detection.orientation.w = sensing_result[0].detection.qw;
		ROS_WARN_STREAM("DETECTED OBJECT IN BOUNDING BOX: "<<sensing_result[0].detection.x<<" "<<sensing_result[0].detection.y<<" "<<sensing_result[0].detection.z<<" "<<sensing_result[0].detection.qx<<" "<<sensing_result[0].detection.qy<<" "<<sensing_result[0].detection.qz<<" "<<sensing_result[0].detection.qw<<" ");
		success = true;
	}
	else
	{
		ROS_ERROR_STREAM("Nothing Detected!");
		success = false;
	}

	return obj_detection;
}



geometry_msgs::Pose TaskPlanner::adjust_pose(bool& success, geometry_msgs::Pose current_pose, geometry_msgs::Pose target_obj_pose, std::string target_bin, std::string object_label)
{
	success = false;
	geometry_msgs::Pose adjusted_pose = current_pose;

	int bin_id;
	if(target_bin=="bin_1")
	{
		bin_id = 3;
	}
	else if(target_bin=="bin_2")
	{
		bin_id = 4;
	}

	// std::vector<double> dimensions = {0.15,0.15,0.1};
	std::vector<double> dimensions = {0.2,0.2,0.1};
	auto bb_pose = current_pose;
	bb_pose.position.z-=0.40;
	bb_pose.position.z-=dimensions[2]/2;
	bb_pose.position.z+=0.03;
	//xyz, qwqxqyqz
	std::vector<double> bb_pose_vec = {bb_pose.position.x, bb_pose.position.y, bb_pose.position.z, 1, 0, 0, 0};
	comm->set_sensing(bin_id, {object_label}, dimensions, bb_pose_vec);
	sleep(SENSING_SLEEP);
	comm->start_sensing();
	sleep(SENSING_SLEEP);
	auto sensing_result = comm->get_sensing();

	geometry_msgs::Pose obj_detection;
	//Blindly trust the detection since we do not reason about the grasps
	if(!sensing_result.empty())
		// && sensing_result[0].end_effector_pose.z>-1000)
	{
		obj_detection.position.x = sensing_result[0].detection.x;
		obj_detection.position.y = sensing_result[0].detection.y;
		obj_detection.position.z = sensing_result[0].detection.z;
		obj_detection.orientation.x = sensing_result[0].detection.qx;
		obj_detection.orientation.y = sensing_result[0].detection.qy;
		obj_detection.orientation.z = sensing_result[0].detection.qz;
		obj_detection.orientation.w = sensing_result[0].detection.qw;
		ROS_WARN_STREAM("DETECTED OBJECT IN BOUNDING BOX: "<<sensing_result[0].detection.x<<" "<<sensing_result[0].detection.y<<" "<<sensing_result[0].detection.z<<" "<<sensing_result[0].detection.qx<<" "<<sensing_result[0].detection.qy<<" "<<sensing_result[0].detection.qz<<" "<<sensing_result[0].detection.qw<<" ");

		if(placement_mod->is_top_surface_graspable(object_label, obj_detection))
		{
			adjusted_pose = placement_mod->get_adjusted_ee_pose(object_label, target_bin, current_pose, obj_detection, target_obj_pose);
			success = true;
		}
		else
		{
			success = false;
		}
	}
	else
	{
		ROS_ERROR_STREAM("Nothing Detected!");
		success = false;
	}

	current_object_detection = obj_detection;
	// std::string xxx;
	// std::cout<<"\nWaiting for keypress to debug bounding box detection...";
	// std::cin>>xxx;

	return adjusted_pose;
}
