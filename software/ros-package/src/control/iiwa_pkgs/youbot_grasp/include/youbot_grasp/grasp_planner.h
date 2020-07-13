/***************************************************************************/
/*      FILE:     grasp_planner.h                                          */
/*      AUTHOR:   Wei Tang                                                 */
/*      DATE:     06-01-2017                                               */
/***************************************************************************/

#ifndef __GRASPPLANNER__
#define __GRASPPLANNER__


#include "approach_dir_generator.h"
#include "grasp_generator.h"
#include "grasp_direction.h"
#include "grasp_grasps.h"
#include "youbot_grasp/PlanningScene.h"


#include <list>
#include <string>
#define DEBUG_

class GraspPlanner
{
private:
  ros::NodeHandle *node_;
  //! A pointer to approach direction generator
  ApproachDirGenerator 			*approach_dir_generator_;

  //! A pointer to grasp generator 
  GraspGenerator 					*grasp_generator_;

  //! A list of approaching direction
  std::list<GraspDirection*> 		 approach_dir_list_;

  //! A list of planned grasp
  std::list<plannedGrasp*> 		 planned_grasps_;

  geometry_msgs::Pose target_pose_;

  youbot_grasp::PlanningScene * planning_scene_;

  std::vector<geometry_msgs::Point> sample_point;
  std::vector<geometry_msgs::Point> sample_normal;
  std::vector<geometry_msgs::Point> grasp_point;
  std::list<sampledPoint*> sampledPoint_list;  // 

  float cup_radius_ = 0.07;
  // type of different grippers, 0 for suction cup, 1 for 3-finger hand
  int gripper_type_ = 0; 
  
public:
	/*
 * constructor, destructor 
 */                  
  GraspPlanner(ros::NodeHandle &node);
  GraspPlanner(ApproachDirGenerator *approach_dir_generator_in, GraspGenerator *grasp_generator_in);
  ~GraspPlanner();

  /*overall planning*/
  bool plan(std::string object_file_name, std::list<plannedGrasp*> & grasp_list);

  /*generate approaching direction*/
  void getApproachDir(std::string object_file_name);

  /*generate preshape for each approaching direction*/
  void getPreshape();

  /*generate final grasps given the approaching direction and preshape*/
  void getGrasp(std::string object_file_name);

  void set_radius(float radius){ cup_radius_ = radius;}

  void set_gripper_type(int gripper_type){ gripper_type_ = gripper_type; }

  void set_planning_scene(youbot_grasp::PlanningScene* planning_scene){planning_scene_ = planning_scene;}

  void set_target_pose(geometry_msgs::Pose target_pose){target_pose_ = target_pose;}
};

#endif