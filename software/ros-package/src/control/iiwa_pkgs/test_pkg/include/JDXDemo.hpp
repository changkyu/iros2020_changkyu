#include <TaskPlanner.hpp>
#include <utils.hpp>

#include <task_planning/SegmentationCall.h>
#include <task_planning/ObjectPose.h>
#include <geometry_msgs/Pose.h>
#include <perception_communication.hpp>
//#include "test_pkg/GraspPlanning.h"
#include "grasping/GraspPlanning.h"
#include <placement_module.hpp>
class Demo {
private:
	
	//Automaton states
	enum State {START, SENSE, PICK_AND_PLACE, FINISH};
	
	//Data members
	State current_state = START;
	iiwa_ros::iiwaRos my_iiwa;
	bool execution_result;

	ros::NodeHandle nh;

	std::vector<geometry_msgs::Pose> object_poses;
	std::vector<geometry_msgs::Pose> object_detections;
	std::vector<std::string> object_labels;
	iiwa_msgs::JointPosition current_joint_position;
	ros::ServiceClient obj_client = nh.serviceClient<task_planning::SegmentationCall>("object_message");
	ros::ServiceClient obj_client_2 = nh.serviceClient<task_planning::SegmentationCall>("object_message_2");
	
	task_planning::SegmentationCall obj_srv;

	geometry_msgs::PoseStamped target_pose;
	geometry_msgs::PoseStamped place_pose_1;
	geometry_msgs::PoseStamped place_pose_2;

	ros::Rate* loop_rate_ = new ros::Rate(20);

	std::string binFile = "package://motion_planning/models/bin.obj";
	std::string source_bin = "bin_1";
	std::string target_bin = "bin_2";

	std::map<std::string, int> object_attempts_map;
	std::vector<std::string> original_object_list;
	std::vector<std::string> object_list;
	
	bool REGRASP;
	double MAX_TIME;
	ros::Timer marker_timer;
	int obj_num_per_layer;

	//The placement module
	placement_module placement_mod;
	//The communication module that interacts with sensing
	perception_communication* comm;


	//Functions


	void reset_object_attempts();
	//INSTANCE
	void get_objects_to_sense(std::vector<std::string>& object_list, int MAX_ATTEMPTS=1000000);
	//YCB
	// void get_objects_to_sense(std::vector<std::string>& object_list, int MAX_ATTEMPTS=1);
	int get_gripper_control(double width);
	
public:
	Demo();
	//Original pick and drop in bin automaton
	bool run_experiment();

	//Enhanced automaton to arrange objects in a grid
	bool run_experiment_grid();
	void fix_using_push_and_pull();
	

	std::string get_state(State current_state);
	void update_object_poses();
	void savePointCloud(std::string bin_name, int count=0);
	TaskPlanner task_planner;
};
