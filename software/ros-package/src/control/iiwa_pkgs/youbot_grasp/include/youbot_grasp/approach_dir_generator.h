/***************************************************************************/
/*      FILE:     ApproachDirGenerator.h                                          */
/*      AUTHOR:   wei.tang@dorabot.com                                                */
/*      DATE:     21-04-2016                                               */
/***************************************************************************/

#ifndef __APPROACH_DIR_GENERATOR_H__
#define __APPROACH_DIR_GENERATOR_H__

#include<vcg/complex/complex.h> 
#include<wrap/io_trimesh/import_stl.h>
#include<wrap/io_trimesh/import_obj.h>
#include<wrap/io_trimesh/export_stl.h>
#include<vcg/complex/algorithms/point_sampling.h>
#include<vcg/complex/algorithms/create/platonic.h>
#include <string>
#include <ros/ros.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "ros/package.h"  
class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public vcg::UsedTypes <vcg::Use<MyVertex>   ::AsVertexType,
                                            vcg::Use<MyEdge>     ::AsEdgeType,
                                           vcg::Use<MyFace>     ::AsFaceType>{};
                                         
class MyVertex  : public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  >{};
class MyFace    : public vcg::Face< MyUsedTypes, vcg::face::FFAdj,  vcg::face::Normal3f, vcg::face::VertexRef, vcg::face::BitFlags, vcg::face::Mark > {};
class MyEdge    : public vcg::Edge<MyUsedTypes>{};
class MyMesh    : public vcg::tri::TriMesh< vector<MyVertex>, vector<MyFace> , vector<MyEdge>  > {};

class sampledPoint{
	public:
		double quality = 0;
		geometry_msgs::Point sample_point_;
		geometry_msgs::Point sample_normal_;

		sampledPoint(geometry_msgs::Point sample_point, geometry_msgs::Point sample_normal);

};


class ApproachDirGenerator
{
  public:
    /*constructor and destructors*/
    ApproachDirGenerator();
    ~ApproachDirGenerator();

    /*generate approaching directions*/
    void generateApproachDir(std::string object_file_name, geometry_msgs::Pose target_pose, std::list<sampledPoint*> &sampledPoint_list, std::vector<geometry_msgs::Point>* sample_point = 0, std::vector<geometry_msgs::Point>* sample_normal = 0);
    double triangle_area(vcg::Point3f a, vcg::Point3f b, vcg::Point3f c);
    vcg::Point3f get_COM(MyMesh & m);

};





#endif