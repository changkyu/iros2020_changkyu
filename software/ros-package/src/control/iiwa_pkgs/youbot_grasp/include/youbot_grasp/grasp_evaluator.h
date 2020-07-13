
/***************************************************************************/
/*      FILE:     GraspEvaluator.h                                          */
/*      AUTHOR:   Wei Tang                                                 */
/*      DATE:     06-01-2017                                               */
/***************************************************************************/

/*! \file
 \manager class for the grasp planning pipeline
*/

#ifndef __GRASP_EVALUATOR_H__
#define __GRASP_EVALUATOR_H__

#include <vcg/space/point3.h>
#include "approach_dir_generator.h"
#include "grasp_grasps.h"
#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/export_obj.h>
class GraspEvaluator
{
private:
	std::string object_file_name_;
public:
	/*constructors and destructors*/
	GraspEvaluator();
	~GraspEvaluator();

	/*evaluate grasps*/
	bool evaluate(std::string object_file_name, std::list<plannedGrasp*> & grasp_list, std::string gripper_type, double radius = 0.0);
	vcg::Point3f get_COM(MyMesh & m);

};

#endif


