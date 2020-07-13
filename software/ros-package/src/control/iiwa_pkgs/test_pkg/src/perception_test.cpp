// #include <perception_communication.hpp>
#include <>

int main(int argc, char**argv)
{
	ros::init(argc,argv,"perception_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;
	// ros::init(argc,argv,"Perception_test");
	ros::ServiceClient segment_client = nh.serviceClient<grasping::GraspPlanning>("/CollisionGraspPlanning");
	grasping::GraspPlanning srv;
	srv.request.target_pose = target_pose.pose;
	int gripper_collision_num = 0;
	if (segment_client.call(srv))
	{
	}
	// ros::AsyncSpinner spinner(1);
	// spinner.start();

	// perception_communication* test_perception = new perception_communication();
	return 0;
}