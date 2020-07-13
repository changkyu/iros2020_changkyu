/***************************************************************************/
/*      FILE:     main.cpp                                         */
/*      AUTHOR:   Cao Chao                                                 */
/*      DATE:     07-01-2016                                               */
/***************************************************************************/

#include <youbot_grasp/youbot_grasp.h>
#include "ros/ros.h"
#include <cstdlib>
#include "youbot_grasp/GraspPlanning.h"

class YoubotGrasp;

bool serviceCB_tmp(youbot_grasp::GraspPlanning::Request &req, youbot_grasp::GraspPlanning::Response &res)
{
	std::cout << "service called" << std::endl;
	//res.grasp.id = "handsome grasp";
	return true;
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "youbot_grasp");
	ros::NodeHandle node_handle;

	youbot_grasp::YoubotGrasp youbot_grasp(node_handle);
	// std::vector<fcl::Vector3<double>> vertices_cube_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_collision_gripper_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_base_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_floor_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Triangle> triangles_base_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_floor_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_cube_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_collision_gripper_ = std::vector<fcl::Triangle>();
	ros::ServiceServer grasp_planning_service = node_handle.advertiseService("GraspPlanning", &youbot_grasp::YoubotGrasp::graspPlanningServiceCB, &youbot_grasp);
	ROS_INFO("Ready to plan grasp.");
	ros::spin();

	return 0;
}

