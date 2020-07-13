
/***************************************************************************/
/*      FILE:     FeasiblityChecker.h                                          */
/*      AUTHOR:   Wei Tang                                                  */
/*      DATE:     06-01-2017                                               */
/***************************************************************************/

#ifndef __FEASIBILITY_CHECKER_H__
#define __FEASIBILITY_CHECKER_H__

#include <sensor_msgs/JointState.h>


class FeasibilityChecker
{
	protected:
    //! robot state
    sensor_msgs::JointState robot_state_;

    //! environment
    //octomap_msgs::Octomap environment_;

	public:
		/*constructors and destructors*/
		FeasibilityChecker();
		~FeasibilityChecker();

		/*check feasibility*/
		bool check();
		
};

#endif