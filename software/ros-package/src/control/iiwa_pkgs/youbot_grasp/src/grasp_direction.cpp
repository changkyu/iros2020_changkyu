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
// $Id: grasp_directions.cpp,v 1.2 2009/03/25 22:10:05 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_directions.cc                                      */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-27-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the class GraspDirection , the palm position and approach vector for a candidate grasp (used in the grasp planner)
*/


#include <math.h>


/* STL */
#include "list"
#include "youbot_grasp/grasp_direction.h"

/*!
  Sets empty to false.
*/
GraspDirection::GraspDirection()
{
  //    point = new coordinates;
  //    dir = new coordinates;
    empty = false;
}

/*!
  Stub destructor.
*/
GraspDirection::~GraspDirection()
{
  //    delete point;
  //    delete dir;
}

/*!
  Returns a pointer to the palm position coordinates.
*/
tf::Vector3      
GraspDirection::get_point()      const
{
    return *point;
}

/*!
  Returns a pointer to the palm approach vector coordinates.
*/
tf::Vector3         
GraspDirection::get_dir()          const
{ 
    return *dir;
}

/*!
  Sets empty to the value of \a in .
*/
void                
GraspDirection::set_empty(bool in)
{
    empty = in;
}

/*!
  Returns the value of empty.
*/
bool                
GraspDirection::get_empty()        const
{
    return empty;
}

/*!
  Sets the grasp direction type to \a in .
*/
void                
GraspDirection::set_gdType(graspDirectionType in)
{
    gdType = in;
}

/*!
  Returns the grasp direction type.
*/
graspDirectionType  
GraspDirection::get_gdType()       const
{
    return gdType;
}

/*!
  Compares this grasp direction to \a p .  If the point and direction
  are the same and the value of empty is the same, then it returns TRUE.  
*/
bool            
GraspDirection::operator==(const GraspDirection& p)
{

  //ATM: Hmmmm why is steffen comparing pointer values?
    if (p.get_point() == get_point() &&
	p.get_dir() == get_dir() &&
	p.get_empty() == get_empty())
	return true;
    return false;
}



/***************
CARTESIAN GraspDirection
***************/

/*!
  Creates new cartesian coordinates for both point and dir . Sets empty to
  FALSE.
*/
cartesianGraspDirection::cartesianGraspDirection()
{
    point = new tf::Vector3(0,0,0);
    dir = new tf::Vector3(0,0,0);
    empty = false;
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cartesianGraspDirection::cartesianGraspDirection(GraspDirection* p)
{
    point = new tf::Vector3(p->get_point().x(),p->get_point().y(),p->get_point().z());
    dir = new tf::Vector3(p->get_dir().x(),p->get_dir().y(),p->get_dir().z());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cartesianGraspDirection::cartesianGraspDirection(cartesianGraspDirection* p)
{
    point = new tf::Vector3(p->get_point().x(),p->get_point().y(),p->get_point().z());
    dir = new tf::Vector3(p->get_dir().x(),p->get_dir().y(),p->get_dir().z());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cartesianGraspDirection::cartesianGraspDirection(const cartesianGraspDirection& p) : GraspDirection()
{
    point = new tf::Vector3(p.get_point().x(),p.get_point().y(),p.get_point().z());
    dir = new tf::Vector3(p.get_dir().x(),p.get_dir().y(),p.get_dir().z());
    empty = p.get_empty();
    set_gdType(p.get_gdType());
}

/*!
  Deletes point and dir .
*/
cartesianGraspDirection::~cartesianGraspDirection()
{
    delete point;
    delete dir;
}

/*!
  Sets the point to the value of \a in .
*/
void        
cartesianGraspDirection::set_point(tf::Vector3 in)
{
    (*point).setX(in.x());
        (*point).setY(in.y());
    (*point).setZ(in.z());

}

/*!
  Sets the directions to the value of \a in .
*/
void        
cartesianGraspDirection::set_dir(tf::Vector3 in)
{
   (*dir).setX(in.x());
        (*dir).setY(in.y());
    (*dir).setZ(in.z());
}





















