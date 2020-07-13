#pragma once
#ifndef IIWA_PKGS_MOTION_PLANNING
#define IIWA_PKGS_MOTION_PLANNING
#include <tuple>
#include <string.h>
#include <iostream>
#include <fstream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <utils.hpp>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>
#include <rosbag/view.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/package.h>

#define foreach BOOST_FOREACH
#define NUM_MIDPTS 10
class MoveItNode {

private:
	ros::NodeHandle node_handle;
	ros::Publisher planning_scene_diff_publisher;
	iiwa_ros::iiwaRos my_iiwa;
	bool success = false;
	geometry_msgs::PoseStamped target_pose;

public:
//	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac_("/iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory", true);
	moveit::planning_interface::MoveGroup group;
	moveit::planning_interface::PlanningSceneInterface plan;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	// moveit::planning_interface::MoveGroup::Plan transfer_plan_1;
	// moveit::planning_interface::MoveGroup::Plan transfer_plan_2;
	moveit::planning_interface::MoveGroup::Plan bin_1_go_plan;
	moveit::planning_interface::MoveGroup::Plan bin_1_back_plan;
	moveit::planning_interface::MoveGroup::Plan bin_2_go_plan;
	moveit::planning_interface::MoveGroup::Plan bin_2_back_plan;
	std::vector<moveit::planning_interface::MoveGroup::Plan> grid_plan_list;
	iiwa_msgs::JointPosition current_joint_position_;
	iiwa_msgs::JointVelocity command_velocity_;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	std::map<std::string, moveit::planning_interface::MoveGroup::Plan> saved_paths;
	std::map<std::tuple<int, int, int, int>, moveit::planning_interface::MoveGroup::Plan> grid_saved_paths;
	std::map<int, geometry_msgs::Pose> bin_1_id_pose_map, bin_2_id_pose_map;
	std::map<int, std::vector<double>> bin_1_grid_map;
	std::map<int, std::vector<double>> bin_2_grid_map;
	std::vector<std::string> saved_path_name_list;
	const double jump_threshold = 0.00;
	const double eef_step = 0.01;
	const double cartesian_plan_limit = 5;
	geometry_msgs::Pose transfer_pose_1;
	geometry_msgs::Pose transfer_pose_2;
	std::vector<double> transfer_joint_1;
	std::vector<double> transfer_joint_2;
	geometry_msgs::Pose bin_1_go_pose;
	geometry_msgs::Pose bin_1_back_pose;
	geometry_msgs::Pose bin_2_go_pose;
	geometry_msgs::Pose bin_2_back_pose;
	std::vector<double> bin_1_go_joint;
	std::vector<double> bin_1_back_joint;
	std::vector<double> bin_2_go_joint;
	std::vector<double> bin_2_back_joint;
	bool pre_computed = false;
	MoveItNode();
	void addCollisionObjectFromMesh(moveit_msgs::CollisionObject& colObj,std::string meshFile, std::vector<float> objPose, std::string bin_id);
	void addCollisionObject(moveit_msgs::CollisionObject& colObj, std::string objId, std::vector<float> objDim, std::vector<float> objPose);
	void addGripperObject();
	bool plan_and_execute(const geometry_msgs::PoseStamped& msg);
	bool plan_and_execute(std::vector<double> robot_state);
	bool goto_home();
	bool quaternion_similarity(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
	bool plan_and_execute_via_waypoints(const geometry_msgs::PoseStamped& target_pose);
	bool plan_and_execute_via_waypoints(const geometry_msgs::PoseStamped& target_pose, bool keep_orientation);
	bool plan_and_execute_via_waypoints(const geometry_msgs::Pose& target_pose);
	double plan_and_execute_via_waypoints(double x, double y, double z, double num_midpts=NUM_MIDPTS);
	double plan_and_execute_via_waypoints(const geometry_msgs::Pose& start_p, const geometry_msgs::Pose& target_p, double num_midpts);
    double plan_and_execute_via_waypoints(const std::vector<geometry_msgs::Pose> &waypoints);
    bool plan_and_execute_U_push(const geometry_msgs::Pose start_p, double x, double y, double z_buf=0.10, double num_midpts=20); //changkyu
	std::vector<double> pre_plan();
	bool execute(std::string source_bin, std::string target_bin, int mode);
	bool execute(std::string source_bin, std::string target_bin, geometry_msgs::Pose target_pose, int mode);
	bool planToStartOfPreC(std::string source_bin, std::string target_bin, int mode);
	bool planToStartOfPreC(std::string source_bin, std::string target_bin, geometry_msgs::Pose target_pose, int mode);
	bool plan_and_execute(const geometry_msgs::Pose& msg);
	void printJointPosition(iiwa_msgs::JointPosition msg);
	bool execute_trajectory(moveit::planning_interface::MoveGroup::Plan trajectory_plan);
	void test_joint_velocity();
	void get_bin_poses(std::vector<float>& bin_pose_1, std::vector<float>& bin_pose_2);
	void convert_wxyz_to_xyzw(std::vector<float>& pose);

	double distance_between_poses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
	int closest_index(geometry_msgs::Pose p, std::string bin);

void PrintMotionPlan(moveit::planning_interface::MoveGroup::Plan plan);
void move_grid(bool);
void pre_plan_grid();
void test_grid_move();
void addBin();
void deleteBin();

};

#endif