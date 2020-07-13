#ifndef UTILS_INLINE__HPP__
#define UTILS_INLINE__HPP__

#include <algorithm>

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#define COS_10 (0.98480775301220800)
#define COS_15 (0.96592582628906830)
#define COS_30 (0.86602540378443860)
#define COS_45 (0.70710678118654757)
#define COS_60 (0.50000000000000000)
#define COS_75 (0.25881904510252074)
#define COS_80 (0.17364817766693041)

template<typename PointT>
static 
void GetPlane( const double normal_x, 
               const double normal_y, 
               const double normal_z, 
               const PointT &pt, pcl::ModelCoefficients &coef )
{
    double norm = sqrt( normal_x*normal_x +
                        normal_y*normal_y +
                        normal_z*normal_z   );

    coef.values.resize(4);
    coef.values[0] = normal_x / norm;
    coef.values[1] = normal_y / norm;
    coef.values[2] = normal_z / norm;
    coef.values[3] = -(coef.values[0]*pt.x + 
                       coef.values[1]*pt.y + 
                       coef.values[2]*pt.z   );
}

template<typename PointT>
static 
void GetPlane( const pcl::Normal normal, const PointT &pt, 
               pcl::ModelCoefficients &coef )
{
    GetPlane(normal.normal_x,normal.normal_y,normal.normal_z,pt,coef);
}

template<typename PointT>
static 
void GetPlane( const Eigen::Vector3f &normal, const PointT &pt, 
               pcl::ModelCoefficients &coef )
{
    GetPlane(normal[0],normal[1],normal[2],pt,coef);
}

template<typename PointT>
static 
void transform( const PointT &pt_in, PointT &pt_out, 
                const Eigen::Matrix4f &tf)
{
    PointT pt;
    pt.x = tf(0,0)*pt_in.x + tf(0,1)*pt_in.y + tf(0,2)*pt_in.z + tf(0,3);
    pt.y = tf(1,0)*pt_in.x + tf(1,1)*pt_in.y + tf(1,2)*pt_in.z + tf(1,3);
    pt.z = tf(2,0)*pt_in.x + tf(2,1)*pt_in.y + tf(2,2)*pt_in.z + tf(2,3);
    pt_out = pt;
}

static 
void transform( const pcl::Normal &nm_in, pcl::Normal &nm_out, 
                const Eigen::Matrix4f &tf)
{
    pcl::Normal nm;
    nm.normal_x = tf(0,0)*nm_in.normal_x + tf(0,1)*nm_in.normal_y + tf(0,2)*nm_in.normal_z;
    nm.normal_y = tf(1,0)*nm_in.normal_x + tf(1,1)*nm_in.normal_y + tf(1,2)*nm_in.normal_z;
    nm.normal_z = tf(2,0)*nm_in.normal_x + tf(2,1)*nm_in.normal_y + tf(2,2)*nm_in.normal_z;
    nm_out = nm;
}

static 
void transform( const Eigen::Vector3f &vec, Eigen::Vector3f &vec_out, 
                const Eigen::Matrix4f &tf)
{
    Eigen::Vector3f out;
    out(0) = tf(0,0)*vec(0) + tf(0,1)*vec(1) + tf(0,2)*vec(2);
    out(1) = tf(1,0)*vec(0) + tf(1,1)*vec(1) + tf(1,2)*vec(2);
    out(2) = tf(2,0)*vec(0) + tf(2,1)*vec(1) + tf(2,2)*vec(2);
    vec_out = out;
}

template<typename PointT1, typename PointT2>
static float compute_dist(const PointT1 &pt1, const PointT2 &pt2)
{
    return sqrt((pt1.x-pt2.x)*(pt1.x-pt2.x) + 
                (pt1.y-pt2.y)*(pt1.y-pt2.y) + 
                (pt1.z-pt2.z)*(pt1.z-pt2.z)  );
}

template <typename PointT> 
static float compute_dist(const pcl::ModelCoefficients &plane, const PointT &pt)
{
    return plane.values[0]*pt.x + 
           plane.values[1]*pt.y + 
           plane.values[2]*pt.z + plane.values[3];
}

template<typename PointT1, typename PointT2, typename PointT3>
static
float compute_dist_Point2Line(const PointT1 &pt, 
                              const PointT2 &pt1, const PointT3 &pt2)
{
    Eigen::Vector3f x0(pt.x,  pt.y,  pt.z), 
                    x1(pt1.x, pt1.y, pt1.z), 
                    x2(pt2.x, pt2.y, pt2.z);
    return (x0-x1).cross(x0-x2).norm() / (x2-x1).norm();
}

template<typename PointT1, typename PointT2, typename PointT3>
static
float compute_dist_Point2LineSegment(const PointT1 &pt, 
                                     const PointT2 &pt1, const PointT3 &pt2)
{
    Eigen::Vector3f x0(pt.x,  pt.y,  pt.z), 
                    x1(pt1.x, pt1.y, pt1.z), 
                    x2(pt2.x, pt2.y, pt2.z);

    Eigen::Vector3f vec = x2-x1;
    vec.normalize();
    float x0_prj = vec.dot(x0);
    float x1_prj = vec.dot(x1);
    float x2_prj = vec.dot(x2);

    if( (x0_prj<x1_prj && x0_prj<x2_prj) || (x0_prj>x1_prj && x0_prj>x2_prj) )
    {
        float dist1 = (x0-x1).norm();
        float dist2 = (x0-x2).norm();
        return std::min(dist1, dist2);
    }
    else
    {
        return (x0-x1).cross(x0-x2).norm() / (x2-x1).norm();
    }
}

#endif