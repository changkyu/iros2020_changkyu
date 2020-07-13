/*
File: JDXDemo.cpp

Authors: Aravind Sivaramakrishnan

Description:

Comments/TODO:
-
-
*/
#pragma once
#ifndef PERCEPTION_COMMUNICATION
#define PERCEPTION_COMMUNICATION

// #define REFLEX
#include "ros/ros.h"
#include <ros/package.h>
#include <vector>
#include <chimp_pose_estimation/GetObjectPose.h>
#include <chimp_pose_estimation/SetObjectNameBinIdMsg.h>
//#include <utils.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SpawnModel.h"
#include <fstream>
//#define USE_GAZEBO
class config_t
{
public:
	double x;
	double y;
	double z;
	double qx;
	double qy;
	double qz;
	double qw;

	config_t(){}
	config_t(double in_x,double in_y,double in_z,double in_qx,double in_qy,double in_qz, double in_qw)
	{
		x = in_x;
		y = in_y;
		z = in_z;
		qx = in_qx;
		qy = in_qy;
		qz = in_qz;
		qw = in_qw;
	}
	config_t(Eigen::Vector3f trans, Eigen::Matrix3f rotm) {
		Eigen::Quaternionf q(rotm);
		x = trans[0];
		y = trans[1];
		z = trans[2];
		qx = q.x();
		qy = q.y();
		qz = q.z();
		qw = q.w();
	}

	std::string print()
	{
		std::string output;
		output+= "[ " +
		std::to_string(x) + " " +
		std::to_string(y) + " " +
		std::to_string(z) + " " +
		std::to_string(qx) + " " +
		std::to_string(qy) + " " +
		std::to_string(qz) + " " +
		std::to_string(qw) + " ]";
		return output;
	}

	geometry_msgs::Pose get_pose()
	{
		geometry_msgs::Pose ret_pose;
		ret_pose.position.x = x;
		ret_pose.position.y = y;
		ret_pose.position.z = z;

		ret_pose.orientation.x = qx;
		ret_pose.orientation.y = qy;
		ret_pose.orientation.z = qz;
		ret_pose.orientation.w = qw;

		return ret_pose;
	}
};

class response_t
{
public:
	config_t end_effector_pose;
	config_t detection;
	std::string object_name;
	double object_width;
	bool regrasp;

	response_t(){}
	response_t(std::string in_name, config_t in_config, double width)
	{
		object_name = in_name;
		end_effector_pose = in_config;
		object_width = width;
		regrasp = false;
	}
	response_t(std::string in_name, config_t in_config, config_t in_detect, double width)
	{
		object_name = in_name;
		end_effector_pose = in_config;
		object_width = width;
		detection = in_detect;
		regrasp = false;
	}
	response_t(std::string in_name, config_t in_config, config_t in_detect, double width, bool in_regrasp)
	{
		object_name = in_name;
		end_effector_pose = in_config;
		object_width = width;
		detection = in_detect;
		regrasp = in_regrasp;
	}
	std::string print()
	{
		std::string output;
		output+="\n######################################";
		output+="\n######################################";
		output+="\n######################################";
		output+="\nObject name: " + object_name;
		output+="\nObject width: " + std::to_string(object_width);
		output+="\nEE Pose: ";
		output+=end_effector_pose.print();
		output+="\nObject Detection: ";
		output+=detection.print();
		output+="\n######################################";
		output+="\n######################################";
		output+="\n######################################";
		return output;
	}
};

class object_surface
{
public:
	Eigen::Vector3f center;
	Eigen::Matrix3f rotation; // x: primary, y: secondary, z: surface normal
	Eigen::Vector2f dimensions;
	bool graspable;

	object_surface(){graspable = true;}
	~object_surface(){}
	Eigen::Vector3f surface_normal() 
	{
		return rotation.col(2);
	}
	Eigen::Vector3f primary_axis() 
	{
		return rotation.col(0);
	}
	Eigen::Vector3f secondary_axis() 
	{
		return rotation.col(1);
	}
	Eigen::Vector3f center_point() 
	{
		return center;
	}

	void print()
	{
		std::cout<<"\n------------------\nSurface-- center:\n"<<center<<"\n rotation:\n"<<rotation<<"\n dimensions:\n"<<dimensions<<"\n graspable: "<<graspable<<"\n";
	}
};

void publish_arrow(const ros::TimerEvent& event);
class perception_communication
{
public:
	perception_communication();
	~perception_communication()
	{}
	
	void set_sensing(int bin_number, std::vector<std::string > object_names)
	{
		set_bin_id(bin_number);
		set_object_list(object_names);
		bounding_box_dimensions = {0,0,0};
		bounding_box_pose = {0,0,0,1,0,0,0};
	}

	void set_sensing(int bin_number, std::vector<std::string > object_names, std::vector<double> bb_dimensions, std::vector<double> bb_pose)
	{
		set_bin_id(bin_number);
		set_object_list(object_names);
		set_bounding_box(bb_dimensions, bb_pose);
	}

	std::tuple<int, std::vector<std::string > > current_sensing_structures()
	{
		return std::make_tuple(bin_id,object_list);
	}

	void start_sensing();
	std::vector<response_t> get_sensing();
	std::vector<response_t> get_sensing_from_object_pose(std::string label, geometry_msgs::Pose pose);

	void extract_surfaces_cuboid(std::string obj_name, pcl::PointCloud<pcl::PointNormal>::Ptr pcl_model);
	void read_object_models();

	object_surface* get_surface(Eigen::Vector3f& position_OBB_vec, std::vector<Eigen::Vector3f>& bounds, 
		int surface_axis, int surface_direction, std::vector<int> other_axes, Eigen::Matrix3f &rotational_matrix_OBB);
	
	double get_dot_product_of_surface_normal(geometry_msgs::Pose object_pose, object_surface* surface, Eigen::Vector3f reference_direction);

	double get_projected_width(geometry_msgs::Pose object_pose, object_surface* surface);

	config_t compute_ee_pose(geometry_msgs::Pose object_pose, object_surface* surface);
	config_t compute_ee_pose(geometry_msgs::Pose object_pose);

	void set_object_list(std::vector<std::string> input_object_list)
	{
		object_list = input_object_list;
	}

	void set_bin_id(int input_bin_id)
	{
		bin_id = input_bin_id;
	}

	void set_bounding_box(std::vector<double> bb_dimensions, std::vector<double> bb_pose)
	{
		bounding_box_dimensions = bb_dimensions;
		bounding_box_pose = bb_pose;
	}

	ros::NodeHandle nh;
	ros::ServiceClient set_estimation_client;
	ros::ServiceClient get_estimation_client;
	// chimp_pose_estimation::SetObjectNameBinId set_estimation_srv;
	ros::Publisher set_estimation_pub;
	chimp_pose_estimation::GetObjectPose get_estimation_srv;
	std::map<std::string, std::vector<object_surface*> > surface_map;
	
	std::vector<object_surface*> scene_surfaces;

	double surface_to_grasp_offset;
	std::vector<std::string> object_list;
	int bin_id;
	std::vector<double> bounding_box_dimensions;
	std::vector<double> bounding_box_pose;


};

void spawnObject(ros::NodeHandle &node_handle_, 
    const std::string &file_name, 
    const std::string &model_name, 
    geometry_msgs::Pose pose);

#endif