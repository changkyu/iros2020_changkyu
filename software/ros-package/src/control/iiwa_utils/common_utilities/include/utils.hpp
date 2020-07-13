#pragma once
#ifndef IIWA_UTILS_COMMON
#define IIWA_UTILS_COMMON

#include <iiwa_ros.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_ros/conversions.h>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SpawnModel.h"
#include <fstream>
#include <ctime>
#include <iomanip>
#define USE_GAZEBO
//#define OBJECT_SOURCE 0
// OBJECT_SOURCE: 0: self-generated in gazebo, 1: pose from pose-estimation module
namespace utilities
{
	void sleepForMotion(iiwa_ros::iiwaRos& iiwa, const double maxSleepTime);
	void goToHomePosition(iiwa_ros::iiwaRos& my_iiwa);
	std::string wait_for_response(std::string prompt="Wait for response...");
	void writeToLog(std::string str);
}

#endif