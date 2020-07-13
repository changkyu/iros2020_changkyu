/***************************************************************************/
/*      FILE:     grasp_planner.cpp                                          */
/*      AUTHOR:   wei.tang@rutgers.com                                                 */
/*      DATE:     06-01-2017                                               */
/***************************************************************************/

#include <youbot_grasp/grasp_planner.h>




GraspPlanner::GraspPlanner(ros::NodeHandle &node)
{
  node_ = &node;
  approach_dir_generator_ = new ApproachDirGenerator();
 
  grasp_generator_ = new GraspGenerator(cup_radius_, node);
    
}

GraspPlanner::GraspPlanner(ApproachDirGenerator *approach_dir_generator_in, GraspGenerator *grasp_generator_in)
{
  approach_dir_generator_ = approach_dir_generator_in;
 
  grasp_generator_ = grasp_generator_in;

}

GraspPlanner::~GraspPlanner()
{
  delete approach_dir_generator_;
  
  delete grasp_generator_;


}

/*overall planning*/
bool GraspPlanner::plan(std::string object_file_name, std::list<plannedGrasp*> & grasp_list)
{
  grasp_generator_->setRadius(cup_radius_);
  getApproachDir(object_file_name);
  // getPreshape();
  getGrasp(object_file_name);
  std::list<plannedGrasp*>::iterator i;
  for (i = planned_grasps_.begin();i != planned_grasps_.end(); i++)
  {
    grasp_list.push_back(*i);
  }
  
}

/*generate approaching direction*/
void GraspPlanner::getApproachDir(std::string object_file_name)
{
	approach_dir_generator_->generateApproachDir(object_file_name, target_pose_, sampledPoint_list, &sample_point, &sample_normal);
}

/*generate preshapes for each approaching direction*/
// void GraspPlanner::getPreshape()
// {
//   preshape_generator_->generatePreshape();
// }

/*generate final grasp*/
void GraspPlanner::getGrasp(std::string object_file_name)
{
  /*generate grasp*/
  grasp_generator_->generateGrasp(object_file_name, sample_point, sample_normal, planned_grasps_, gripper_type_, planning_scene_, target_pose_, sampledPoint_list); 	
}



