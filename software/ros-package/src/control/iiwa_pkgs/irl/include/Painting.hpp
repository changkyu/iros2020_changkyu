#include <TaskPlanner.hpp>
#include <utils.hpp>

#include <task_planning/SegmentationCall.h>
#include <task_planning/ObjectPose.h>


class Painting {
private:
	TaskPlanner task_planner;

	enum State {START, PICK_BRUSH, PRE_DIP, DIP, POST_DIP, AIM, TOUCH_CANVAS, PAINT, FINISH};
	State current_state = START;

	std::map< std::string, std::vector<double> > object_poses;

	std::vector<double> offset;

	bool execution_result;

	ros::NodeHandle nh;

	ros::ServiceClient obj_client = nh.serviceClient<task_planning::SegmentationCall>("object_message");
	task_planning::SegmentationCall obj_srv;

	geometry_msgs::PoseStamped grasp_pose;
	geometry_msgs::PoseStamped target_pose;

	ros::Rate* loop_rate_ = new ros::Rate(0.5);
public:
	Painting();
	bool run_experiment();
	std::string get_state(State current_state);
	void set_target(std::string object);
	void update_object_poses();
};