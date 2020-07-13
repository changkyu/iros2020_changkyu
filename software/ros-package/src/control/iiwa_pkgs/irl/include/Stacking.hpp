#include <TaskPlanner.hpp>
#include <utils.hpp>

#include <task_planning/SegmentationCall.h>
#include <task_planning/ObjectPose.h>

class Stacking {
private:
	TaskPlanner task_planner;

	enum State {START, SENSE, WAIT_FOR_PREDICTION, PICK, PLACE, FINISH};
	State current_state = START;

	bool execution_result;
	std::string prediction_result;

	ros::NodeHandle nh;

	std::map< std::string, std::vector<double> > object_poses;
	std::map< std::string, std::vector<double> > object_sizes;

	ros::ServiceClient obj_client = nh.serviceClient<task_planning::SegmentationCall>("object_message");
	task_planning::SegmentationCall obj_srv;

	geometry_msgs::PoseStamped target_pose;

	ros::Rate* loop_rate_ = new ros::Rate(0.5);
public:
	Stacking();
	bool run_experiment();
	std::string get_state(State current_state);
	void update_object_poses_and_sizes();
	void set_target(std::string object);
};