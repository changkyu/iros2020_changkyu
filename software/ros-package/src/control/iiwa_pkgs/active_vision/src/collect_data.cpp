#include <data_collection.hpp>

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "active_vision_data_collection");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	CollectData data_collector;

	geometry_msgs::PoseStamped s_p = data_collector.task_planner.get_current_pose();
	geometry_msgs::PoseStamped e_p = s_p;

	s_p.pose.position.z -= 0.35;
	e_p.pose.position.z -= 0.35;

	s_p.pose.position.x += 0.1;
	s_p.pose.position.y += 0.1;

	data_collector.push(s_p,e_p);

	s_p.pose.position.y -= 0.2;

	data_collector.push(s_p,e_p);

	return 0;
}