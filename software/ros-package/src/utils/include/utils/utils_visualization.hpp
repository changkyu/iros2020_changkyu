#ifndef _UTILS_VISUALIZATION__HPP_
#define _UTILS_VISUALIZATION__HPP_

#include <pcl/visualization/cloud_viewer.h>

namespace utils
{

template<typename PointT>
void addCube(pcl::visualization::PCLVisualizer &viewer,
             const PointT &pt_min, const PointT &pt_max,
             double r=1.0, double g=1.0, double b=1.0, 
             const std::string &id="cube", int viewport=0);


void addPointCloud( pcl::visualization::PCLVisualizer &viewer, 
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                    const std::string &id="cloud", int viewport=0);

void addVoxel( pcl::visualization::PCLVisualizer &viewer, 
               const voxel_t &voxel, const float resolution=0.005,               
               const std::string &id="voxel", int viewport=0, 
               int target=0, bool only_pos=false );

void addTSDF(  pcl::visualization::PCLVisualizer &viewer, 
               const tsdf_t &tsdf, const float resolution=0.005,
               const std::string &id="tsdf", int viewport=0  );

void addWorkspace( pcl::visualization::PCLVisualizer &viewer, 
                   const std::vector<float> &workspace,
                   double r=1, double b=1, double g=1,
                   const std::string &id="workspace", int viewport=0 );
}

#endif