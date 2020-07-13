
/***************************************************************************/
/*      FILE:     youbot_grasp.h                                              */
/*      AUTHOR:   Wei Tang                                                 */
/*      DATE:     06-01-2017                                               */
/***************************************************************************/

/*! \file
 \manager class for the grasp planning pipeline
*/


#ifndef __YOUBOT_GRASP_H__
#define __YOUBOT_GRASP_H__


#include "grasp_planner.h"
#include "feasibility_checker.h"
#include "grasp_evaluator.h"
#include "youbot_grasp/GraspPlanning.h"  // service file

#include <eigen3/Eigen/Dense>


#include <sensor_msgs/JointState.h>
#include <youbot_grasp/PlannedGrasp.h>
#include <youbot_grasp/PlannedGrasp_vector.h>

namespace youbot_grasp{
  

class YoubotGrasp
{
private:

	ros::NodeHandle* node_;
	//! A pointer to the grasp planner
	GraspPlanner *grasp_planner_;

	//! A pointer to the feasibility checke*grasp_planner_
	FeasibilityChecker *feasibility_checker_;

	//! A pointer to the grasp evaluator
	GraspEvaluator *grasp_evaluator_;

	//! A list of planned grasp
	std::list<plannedGrasp*> grasp_list_;

	//object file name
	std::string object_file_name_;

	//object pose
	Eigen::Vector3d object_pose;

	//object position
	Eigen::Vector3d object_position;

	//environment representation
	//moveit_msgs::PlanningScene planning_scene_env_;

	//robot representatoin
	//robot_state::RobotState* robot_state_ptr_;

	
public:
/*
 * constructor, destructor 
 */
    YoubotGrasp(ros::NodeHandle &node);
    YoubotGrasp(GraspPlanner *grasp_planner_in, FeasibilityChecker *feasibility_checker_in, GraspEvaluator *grasp_evaluator_in);
    ~YoubotGrasp();

    /*call back function for the whole grasp planning pipe line*/
    bool graspPlanningServiceCB(youbot_grasp::GraspPlanning::Request &req, youbot_grasp::GraspPlanning::Response &res);
    
    /*grasp planning*/
    bool planGrasp();

    /*feasibility checking*/
    bool checkFeasibility();

    /*grasp evaluation*/
    void evaluateGrasp();

    /*grasp clustering*/
    void clusterGrasp();

    /*convert data from request message*/
    void convertFromMessage();

    /*convert fata to response message*/
    void convertToMessage();
    
    bool compare_quality(const plannedGrasp* first, const plannedGrasp* second);

};


}







#endif