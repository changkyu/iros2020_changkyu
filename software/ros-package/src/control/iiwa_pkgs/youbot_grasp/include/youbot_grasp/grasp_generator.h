/***************************************************************************/
/*      FILE:     GraspGenerator.h                                          */
/*      AUTHOR:   Cao Chao                                                 */
/*      DATE:     07-01-2016                                               */
/***************************************************************************/

#ifndef __GRASP_GENERATOR_H__
#define __GRASP_GENERATOR_H__
#include <vcg/complex/complex.h> 
#include <wrap/io_trimesh/import_stl.h>
#include <wrap/io_trimesh/export_stl.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/intersection.h>
#include <vcg/complex/algorithms/inside.h>
#include <ros/ros.h>
#include <unistd.h>
#include <vcg/space/point3.h>
#include "approach_dir_generator.h"
#include "grasp_grasps.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
// #include "fcl/math/bv/utility.h"
// #include "fcl/narrowphase/collision.h"
// #include "fcl/common/types.h"
// #include "fcl/common/unused.h"

// #include "fcl/math/constants.h"
// #include "fcl/math/triangle.h"

// #include "fcl/geometry/shape/box.h"
// #include "fcl/geometry/shape/sphere.h"
// #include "fcl/geometry/shape/cylinder.h"
// #include "fcl/geometry/bvh/BVH_model.h"
// #include "fcl/geometry/octree/octree.h"

// #include "fcl/narrowphase/collision.h"
// #include "fcl/narrowphase/distance.h"
// #include "fcl/narrowphase/collision_object.h"
// #include "fcl/narrowphase/collision_result.h"
// #include "fcl/narrowphase/continuous_collision_object.h"
// #include "fcl/narrowphase/continuous_collision_request.h"
// #include "fcl/narrowphase/continuous_collision_result.h"
#include <vector>
#include "ros/package.h"
#include "youbot_grasp/PlanningScene.h"

#include "fcl/collision.h"
#include "fcl/collision_object.h"
#include "fcl/config.h"
#include "fcl/collision_data.h"
#include "fcl/collision_node.h"
#include "fcl/math/matrix_3f.h"
#include "fcl/math/transform.h"
#include "fcl/math/vec_3f.h"
#include "fcl/BVH/BVH_model.h"
#define DEBUG_

class GraspGenerator
{
private:
	float radius = 1;
  ros::NodeHandle * node_;
public:


/*
 * constructor, destructor 
 */
  GraspGenerator(float R, ros::NodeHandle &node);
  ~GraspGenerator();

  /*generate the grasp*/
  int generateGrasp(std::string object_file_name, std::vector<geometry_msgs::Point>& sample_point , std::vector<geometry_msgs::Point>& sample_normal , std::list<plannedGrasp*>& grasp_list, int gripper_type, youbot_grasp::PlanningScene * planning_scene, geometry_msgs::Pose target_pose, std::list<sampledPoint*> sampledPoint_list);
  void setRadius(float r);
  tf::Matrix3x3 get_cp_obj_quaternion(geometry_msgs::Point grasp_point, geometry_msgs::Point grasp_dir,double rotate_around_x);
  void generateCollisionPoints(tf::Transform palm_obj_tf, std::vector<tf::Vector3>& collision_points,int gripper_type, double palm_fallback);
  static bool checkCollision(std::string object_1, std::string object_2, fcl::Vec3f T1, fcl::Vec3f T2, fcl::Matrix3f R1, fcl::Matrix3f R2);
  static void geoPoseToFclPose(geometry_msgs::Pose p, fcl::Vec3f &v, fcl::Matrix3f &m);
  void turnFromLocalToGlobal(tf::Matrix3x3 local_to_global_m, double local_x, double local_y, double local_z, double& global_x, double& global_y, double& global_z);
  static void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles);
  static void loadFile();


  // static std::vector<fcl::Vector3<double>> vertices_cube_ = std::vector<fcl::Vector3<double>>(0);
  // static std::vector<fcl::Vector3<double>> vertices_collision_gripper_ = std::vector<fcl::Vector3<double>>(0);
  // static std::vector<fcl::Vector3<double>> vertices_base_ = std::vector<fcl::Vector3<double>>(0);
  // static std::vector<fcl::Vector3<double>> vertices_floor_ = std::vector<fcl::Vector3<double>>(0);
  // static std::vector<fcl::Triangle> triangles_base_ = std::vector<fcl::Triangle>(0);
  // static std::vector<fcl::Triangle> triangles_floor_ = std::vector<fcl::Triangle>(0);
  // static std::vector<fcl::Triangle> triangles_cube_ = std::vector<fcl::Triangle>(0);
  // static std::vector<fcl::Triangle> triangles_collision_gripper_ = std::vector<fcl::Triangle>(0);


  static std::vector<fcl::Vec3f> vertices_cube_;
  static std::vector<fcl::Triangle> triangles_cube_;
  static std::vector<fcl::Vec3f> vertices_floor_;
  static std::vector<fcl::Triangle> triangles_floor_;
  static std::vector<fcl::Vec3f> vertices_collision_gripper_;
  static std::vector<fcl::Triangle> triangles_collision_gripper_;
  static std::vector<fcl::Vec3f> vertices_base_;
  static std::vector<fcl::Triangle> triangles_base_;
  static std::vector<fcl::Vec3f> vertices_opti_cube_;
  static std::vector<fcl::Triangle> triangles_opti_cube_;
  static std::vector<fcl::Vec3f> vertices_standard_gripper_;
  static std::vector<fcl::Triangle> triangles_standard_gripper_;

  static std::vector<fcl::Vec3f> vertices_arm0_;
  static std::vector<fcl::Triangle> triangles_arm0_;
  static std::vector<fcl::Vec3f> vertices_arm1_;
  static std::vector<fcl::Triangle> triangles_arm1_;
  static std::vector<fcl::Vec3f> vertices_arm2_;
  static std::vector<fcl::Triangle> triangles_arm2_;
  static std::vector<fcl::Vec3f> vertices_arm3_;
  static std::vector<fcl::Triangle> triangles_arm3_;
  static std::vector<fcl::Vec3f> vertices_arm4_;
  static std::vector<fcl::Triangle> triangles_arm4_;
  static std::vector<fcl::Vec3f> vertices_arm5_new_;
  static std::vector<fcl::Triangle> triangles_arm5_new_;
};

#endif