/***************************************************************************/
/*      FILE:     youbot_grasp.cpp                                         */
/*      AUTHOR:   wei.tang@rutgers.edu                                                 */
/*      DATE:     06-01-2017                                               */
/***************************************************************************/

#include <youbot_grasp/youbot_grasp.h>

//  #define _DEBUG

youbot_grasp::YoubotGrasp::YoubotGrasp(ros::NodeHandle &node)
{
  node_ = &node;
  grasp_planner_ = new GraspPlanner(node);
  feasibility_checker_ = new FeasibilityChecker();
  grasp_evaluator_ = new GraspEvaluator();

#ifdef _DEBUG
  std::cout << "DBG: YoubotGrasp built." << std::endl;
#endif
}

youbot_grasp::YoubotGrasp::YoubotGrasp(GraspPlanner *grasp_planner_in, FeasibilityChecker *feasibility_checker_in, GraspEvaluator *grasp_evaluator_in)
{
  grasp_planner_ = grasp_planner_in;
  feasibility_checker_ = feasibility_checker_in;
  grasp_evaluator_ = grasp_evaluator_in;
#ifdef _DEBUG
  std::cout << "DBG: YoubotGrasp built." << std::endl;
#endif
}

youbot_grasp::YoubotGrasp::~YoubotGrasp()
{
  delete grasp_planner_;
  delete feasibility_checker_;
  delete grasp_evaluator_;
#ifdef _DEBUG
  std::cout << "DBG: YoubotGrasp destroyed." << std::endl;
#endif
}

/*call back function for the whole grasp planning pipe line*/
bool youbot_grasp::YoubotGrasp::graspPlanningServiceCB(youbot_grasp::GraspPlanning::Request &req, youbot_grasp::GraspPlanning::Response &res)
{
  std::cout << "service called" << std::endl;
  grasp_list_.clear();
  if(req.gripper_type.data == "luh_standard")
  { 
    object_file_name_ = req.object_file_name.data;
    grasp_planner_->set_gripper_type(2);// 0 is for suction cup, defined in grasp_planner.h
  }else if(req.gripper_type.data == "standard"){
    object_file_name_ = req.object_file_name.data;
    grasp_planner_->set_gripper_type(3);    
  }
    grasp_planner_->set_radius(req.radius);
    grasp_planner_->set_planning_scene(&(req.planning_scene));
    grasp_planner_->set_target_pose(req.object_pose);
    grasp_planner_->plan(object_file_name_, grasp_list_);// obtain candidate grasp points
    ROS_INFO_STREAM("finished grasp planning, next evaluating");
    //grasp_evaluator_->evaluate(object_file_name_,  grasp_list_, "standard",req.radius);// rank the candidate points by their grasp quality.
    std::list<plannedGrasp*>::iterator i;
    youbot_grasp::PlannedGrasp_vector grasp_vector_msg;
    for(i = grasp_list_.begin(); i != grasp_list_.end(); i ++)  
    {
      youbot_grasp::PlannedGrasp grasp_msg;
      (*i)->set_grasp_msg(grasp_msg);
      grasp_vector_msg.vector.push_back(grasp_msg);
    }
    res.planned_grasp_vector = grasp_vector_msg;

  

	
  std::cout << "object file name: " << req.object_file_name << std::endl;
  std::cout << "object pose: " << std::endl;
  std::cout << "position x: " << req.object_pose.position.x << std::endl;
  std::cout << "position y: " << req.object_pose.position.y << std::endl;
  std::cout << "position z: " << req.object_pose.position.z << std::endl;
    
	
  return true;
}

/*grasp planning*/
bool youbot_grasp::YoubotGrasp::planGrasp()
{
	//grasp_planner_->plan(object_file_name_, std::list<plannedGrasp*> & grasp_list);
}

/*feasibility checking*/
bool youbot_grasp::YoubotGrasp::checkFeasibility()
{
  feasibility_checker_->check();
}

/*grasp evaluation*/
void youbot_grasp::YoubotGrasp::evaluateGrasp()
{
	//grasp_evaluator_->evaluate();
}

/*convert data from request message*/
void youbot_grasp::YoubotGrasp::convertFromMessage()
{

}

/*convert data to response message*/
void youbot_grasp::YoubotGrasp::convertToMessage()
{
	
}

bool youbot_grasp::YoubotGrasp::compare_quality(const plannedGrasp* first, const plannedGrasp* second)
{
  if(first->get_quality() <= second->get_quality())
  {
    return true;
  }else{
    return false;
  } 
}