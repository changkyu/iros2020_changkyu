#pragma once

#ifndef IIWA_PKGS_TASK_PLANNER
#define	IIWA_PKGS_TASK_PLANNER


#include <MoveItNode.hpp>
#include <Grasper.hpp>
#include <utils.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <moveit/robot_state/robot_state.h>
#include <perception_communication.hpp>
#include <placement_module.hpp>
//#define USE_SIMULATION
//#define USE_PUSHING

class TaskPlanner {
private:
	ros::NodeHandle node_handle;
	// ros::Publisher gripper_command_publisher;

	bool isGripperActive = false;

	double goal_tolerance = 0.05;

	iiwa_ros::iiwaRos my_iiwa;
	MoveItNode moveit_node;
	GraspNode grasp_node;

	robotiq_s_model_control::SModel_robot_input::ConstPtr gripperStatusMsg;
	// std_msgs::String gripper_command;
	geometry_msgs::PoseStamped transfer_pose_1;
	geometry_msgs::PoseStamped transfer_pose_2;
	geometry_msgs::PoseStamped test_pose_1;
	geometry_msgs::PoseStamped test_pose_2;
	geometry_msgs::PoseStamped pre_place_point;
	std::vector<double> transfer_joint_1;
	std::vector<double> transfer_joint_2;
	std::vector<double> bin_1_back_joint;
	std::vector<double> bin_2_back_joint;
	std::vector<double> bin_1_go_joint;
	std::vector<double> bin_2_go_joint;	
	double pre_place_offset = 0.10;
	iiwa_msgs::JointVelocity command_velocity_;
	perception_communication* comm;
	
	
public:
	TaskPlanner();
	TaskPlanner(std::string gripperMode);
	
	//High level function that tries to pick the object at the grasp point and drop in the target_obj_pose, derived from the placement module 
	//The arguments:
	// grasp_point : the world frame R3 point where a top down grasp would be attempted. This specifies a SE3 pose for the end-effector
	// drop_pose : the world frame SE3 pose of the end-effector that achieves the target_obj_pose placement
	// grasped_obj_pose : the world frame SE3 pose of the object at the point before the grasp, as detected by pose estimation
	// target_obj_pose : the world frame SE3 pose of the object derived from the reasoning in the placement module that reports incremental grid positions inside the target bin for the object
	// size :  deprecated interface requirement from grasping module that needed the size of the object for fingered grasps
	// source_bin : the string bin_n describing the n'th bin as the source
	// target_bin : the string bin_n describing the n'th bin as the target
	// object_label : name of the object that is currently being picked up. This is maintained as consistent to obj_config.yaml in chimp_resources
	// REGRASP : the flag that signifies that sensing reported the current object grasp as being on a face that disallows target_obj_pose at the end of placement. This automatically invokes the toppling action
	//
	bool pick_and_drop(const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP = false);
	
	//The function invokes the toppling primitive that 
	// 1. scans the point cloud to compute an empty placement plane position 
	// 2. reasons about the secondary axis of the object to compute a topple direction
	// 3. executes a toppling primitive that lowers the object, drags it laterally in the topple direction and releases it
	void topple(std::string bin, std::string object_label);


	// This function is used to verify, using sensing, after the object has been grasped, whether (return values and their false conditions):
	// return 0 unless: The object is still attached to the end-effector
	// return -1 unless: The object is attached with the expected face that allows placement
	// return 0 unless: The detected object pose after lifting does not differ appreciably from the initial object pose that generated that grasp. This is sympomatic of erroneous pose detection or unstable grasps, both of which are not desirable for robust placement.
	// return 1: if stable correct attachment is detected 
	int is_object_attached(std::string current_bin, std::string object_label, geometry_msgs::Pose grasped_obj_pose, bool check_for_sensing=true );
	

	//This function invokes the push-to-place primitive that uses the point cloud to generate a placement offset and pushes against the surrounding objects to guarantee correct placement
	//The arguments:
	// 	target_obj_pose : The desired object pose coming from the placement module, and corresponds to a grid location in the target bin
	// 	pre_push_pose : The placeplan action server is called inside pick_and_drop() to generate the pre_push_pose that offsets the target_obj_pose, away from the observed point cloud that surrounds the target_obj_pose. Once the object is placed in this offset pose which is clear of the surrounding objects, a pushing action at the same Z plane as the target object, ensures correct placement
	// 	z_offset : The parameter that determines the amount the end-effector is raised or lowered during the primitive. The assumption is that before the module is invoked, the end-effector is raised by the same offset from the desired placement plane.
	// 	target_bin : String that corresponds the the target bin
	// 	object_label : String that corresponds the the current object
	bool adjusted_drop(geometry_msgs::Pose target_obj_pose, geometry_msgs::Pose pre_push_pose,double z_offset, std::string target_bin, std::string object_label);
	bool adjusted_drop_with_tightenv(geometry_msgs::Pose target_obj_pose, std::vector<geometry_msgs::Pose>  pre_push_pose_list,double z_offset, std::string target_bin, std::string object_label);
	



	void getGripperStatus();
	void addCollisionObject(std::string objId, std::vector<float> objDim, std::vector<float> objPose);
	//void addCollisionObjectFromMesh(std::string meshFile, std::vector<float> objPose);
	geometry_msgs::PoseStamped get_current_pose();
	bool checkIfReached(geometry_msgs::PoseStamped target_pose);

	bool pick_and_drop_without_push(const geometry_msgs::PoseStamped& grasp_point, geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP);
	bool pick_and_drop_without_topple(const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP);
	bool pick_and_drop_without_pose_estimation(const geometry_msgs::PoseStamped& grasp_point, geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP);
	bool pick_and_drop_with_tightenv(const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& drop_pose, geometry_msgs::Pose grasped_obj_pose, geometry_msgs::Pose target_obj_pose,double size, std::string source_bin, std::string target_bin, std::string object_label, bool REGRASP);

	bool executeTask(std::string query, const geometry_msgs::PoseStamped& grasp_point, const geometry_msgs::PoseStamped& target_pose, double size, std::string source_bin, std::string target_bin);
	bool executeTask(std::string query);
	bool executeTask(std::string query, const geometry_msgs::PoseStamped& target_pose, double size = 0);
	bool executeTask(std::string query, const std::vector<double>& target_pose);
	bool executeTask(std::string query,  std::string source_bin, std::string target_bin, int mode);
	bool planToStartOfPreC(std::string query,  std::string source_bin, std::string target_bin, int mode);
	bool executeTask(std::string query,  std::string source_bin, std::string target_bin, geometry_msgs::Pose target_pose, int mode);
	bool planToStartOfPreC(std::string query,  std::string source_bin, std::string target_bin, geometry_msgs::Pose target_pose, int mode);
	void delayed_release_event(const ros::TimerEvent& event);
	void delayed_release(double delay);
	bool is_object_attached(std::string current_bin);
	geometry_msgs::PoseStamped get_orthogonal_pose(geometry_msgs::PoseStamped input_pose);
	void test_joint_velocity();
	void validateCalibration();
	bool adjusted_drop_without_push(geometry_msgs::Pose target_obj_pose, geometry_msgs::Pose  pre_push_pose,double z_offset, std::string target_bin, std::string object_label);
	
	geometry_msgs::Pose sense_pose(bool& success, geometry_msgs::Pose current_pose, std::string target_bin, std::string object_label);
	geometry_msgs::Pose adjust_pose(bool& success, geometry_msgs::Pose current_pose, geometry_msgs::Pose target_obj_pose, std::string target_bin, std::string object_label);
	ros::Timer release_timer;
	placement_module* placement_mod;

	geometry_msgs::Pose current_object_detection;

	GraspNode* get_grasp_node()
	{
		return &grasp_node;
	}
	MoveItNode* get_planning_node()
	{
		return &moveit_node;
	}

	enum PRECOMPUTED_MODE
	{
		PREGRASP_TO_DROP, DROP_TO_PREGRASP
	};

	void set_communication_link(perception_communication* input_comm)
	{
		comm = input_comm;
	}

	void set_placement_module(placement_module* input_place)
	{
		placement_mod = input_place;
	}

	std::string print_pose(geometry_msgs::Pose p)
	{
		std::stringstream ss;
		ss<<"[ (";
		ss<<p.position.x<<" ";
		ss<<p.position.y<<" ";
		ss<<p.position.z;
		ss<<") <";
		ss<<p.orientation.x<<" ";
		ss<<p.orientation.y<<" ";
		ss<<p.orientation.z<<" ";
		ss<<p.orientation.w;
		ss<<">";
		return ss.str();
	}


	int regrasp_failure;

};


#endif