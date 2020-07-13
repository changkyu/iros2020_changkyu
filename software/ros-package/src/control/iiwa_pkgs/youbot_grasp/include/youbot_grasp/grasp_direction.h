//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Authors: Steffen Knoop
//          Andrew T. Miller 
//
// $Id: grasp_directions.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_directions.h                                       */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-27-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines the class GraspDirection , the palm position and approach vector for a candidate grasp (used in the grasp planner)
*/

#ifndef __GRASP_DIRECTIONS_H__
#define __GRASP_DIRECTIONS_H__
#include <tf/transform_datatypes.h>


enum graspDirectionType{GDT_CYL_SIDES, GDT_CYL_TOP_BOTTOM, 
			GDT_CUBE_WIDTH, 
			GDT_CUBE_HEIGHT, 
			GDT_CUBE_DEPTH, 
			GDT_SPH, 
			GDT_CONE_TOP, GDT_CONE_BOTTOM, GDT_CONE_SIDE_PLANE, GDT_CONE_EDGE};



//!  This is the parent class for the different types of grasp directions
/*!
  A grasp directions consists of a palm position and palm approach vector.
  These can be expressed in any coordinate system (cartesian, cylindrical,
  or spherical.  A grasp direction is created by the planner by using
  different rules for different types of shape primitives.
*/
class GraspDirection
{
protected:
  //! Palm position
  tf::Vector3 *       point;

  //! Palm approach vector
  tf::Vector3 *       dir;

  //! Not used.  (Not sure what Steffen intended this for)
  bool               empty;

  //! The rule type that resulted in this grasp direction
  graspDirectionType gdType;
  
public:
  GraspDirection();
  virtual ~GraspDirection();
  
  /*! Sets the value of the palm position to \a in . */
  virtual void        set_point(tf::Vector3 in) = 0;
  tf::Vector3         get_point() const;
  
  /*! Sets the value of the palm approach vector to \a in .*/
  virtual void        set_dir(tf::Vector3 in) = 0;
  tf::Vector3         get_dir() const;
  
  void                set_empty(bool in);
  bool                get_empty() const;
  
  void                set_gdType(graspDirectionType);
  graspDirectionType  get_gdType() const; 
  
  bool                operator==(const GraspDirection&);

};

//! This class is used when the palm position and direction are expressed in cartesian coordinates
class cartesianGraspDirection :
public GraspDirection
{
public:
  cartesianGraspDirection();
  cartesianGraspDirection(GraspDirection*);
  cartesianGraspDirection(cartesianGraspDirection*);
  cartesianGraspDirection(const cartesianGraspDirection&);
  
  ~cartesianGraspDirection();
  
  void        set_point(tf::Vector3 in);
  void        set_dir(tf::Vector3 in);

};

//! This class is used when the palm position and direction are expressed in cylindrical coordinates


#endif


/******************
   Local Variables:
   mode:c++
   End:
******************/















