#pragma once

#ifndef IIWA_PLACEMENT_MODULE
#define	IIWA_PLACEMENT_MODULE

// #define REFLEX
#include "ros/ros.h"
#include <ros/package.h>
#include <vector>
//#include <chimp_pose_estimation/GetObjectPose.h>
//#include <chimp_pose_estimation/SetObjectNameBinIdMsg.h>
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
#include <perception_communication.hpp>

//#define USE_GAZEBO


class placement_module
{
public:
	placement_module()
	{
		bin_counters["bin_1"] = 0;
		bin_counters["bin_2"] = 0;
		layer = 0;
	}
	~placement_module(){}

	int setup(std::string);
	geometry_msgs::Pose get_grid_pose(std::string target_bin, int grid_id);
	geometry_msgs::Pose get_target_pose(std::string target_bin);
	geometry_msgs::Pose get_target_pose(std::string label, geometry_msgs::Pose object_pose, std::string target_bin);
	std::pair<geometry_msgs::Pose, geometry_msgs::Pose> get_target_pose(std::string label, geometry_msgs::Pose object_pose, geometry_msgs::Pose grasp_pose, std::string target_bin);
	geometry_msgs::Pose get_adjusted_ee_pose(std::string label, std::string target_bin, geometry_msgs::Pose current_ee, geometry_msgs::Pose current_object, geometry_msgs::Pose target_object);
	std::pair< geometry_msgs::Pose, std::vector< std::vector<geometry_msgs::Pose> > > get_adjusted_regrasp_pose(std::string label, std::string source_bin, geometry_msgs::Pose current_ee, geometry_msgs::Pose current_object, geometry_msgs::Pose target_plane_point, double target_alignment_angle);
	geometry_msgs::Pose get_adjusted_aligned_ee_pose(std::string label, std::string source_bin, geometry_msgs::Pose current_ee, geometry_msgs::Pose current_object, double target_alignment_angle);

	bool is_top_surface_graspable(std::string label, geometry_msgs::Pose current_object);
	double check_primary_axis(std::string label, geometry_msgs::Pose object_pose);
	
	std::pair<double, object_surface* > check_primary_axis(std::string label, geometry_msgs::Pose object_pose, bool ret_surface);




	void increment_counter_on_success(std::string target_bin)
	{
		// bin_counters[target_bin] = std::min(bin_counters[target_bin]-1,0);

		int count = -1;
		for(auto key: bin_id_pose_map[target_bin])
			count++;


		if(bin_counters[target_bin]+1>count)
		{
			bin_counters[target_bin] = 0;
			layer++;
		}
		else
			bin_counters[target_bin]++;
	}

protected:
	int get_bin_id(std::string bin_id_str)
	{
		if(bin_id_str=="bin_1")
			return 1;
		else
			return 2;
	}

	void extract_surfaces_cuboid(std::string obj_name, pcl::PointCloud<pcl::PointNormal>::Ptr pcl_model);
	object_surface* get_surface(Eigen::Vector3f& position_OBB_vec, std::vector<Eigen::Vector3f>& bounds, 
	int surface_axis, int surface_direction, std::vector<int> other_axes, Eigen::Matrix3f &rotational_matrix_OBB);
	void read_object_models();
	double get_dot_product_of_surface_normal(geometry_msgs::Pose object_pose, object_surface* surface, Eigen::Vector3f reference_direction);
	double get_distance(Eigen::Vector3f p1, Eigen::Vector3f p2);


	std::map<std::string, std::vector<object_surface*> > surface_map;
	std::map<std::string,std::vector< Eigen::Vector3f > > grasp_normals;
	std::vector<object_surface*> scene_surfaces;
	ros::NodeHandle nh;
	double computed_rotation_about_z, computed_alignment;

	std::map<std::string, std::map<int, geometry_msgs::Pose> > bin_id_pose_map;
	std::map<std::string,int> bin_counters;
	geometry_msgs::Pose object_pose_on_grid;

	double obj_width;
	double obj_length;
	int layer;
};

#endif



