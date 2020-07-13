#include "utils/utils.hpp"
#include "utils/utils_visualization.hpp"
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;

namespace utils
{

template<typename PointT>
void addCube(pcl::visualization::PCLVisualizer &viewer,
             const PointT &pt_min, const PointT &pt_max,
             double r, double g, double b, 
             const std::string &id, int viewport)
{
    PointT pts[8];

    pts[0].x = pt_min.x; pts[0].y = pt_min.y; pts[0].z = pt_min.z;
    pts[1].x = pt_max.x; pts[1].y = pt_min.y; pts[1].z = pt_min.z;
    pts[2].x = pt_max.x; pts[2].y = pt_max.y; pts[2].z = pt_min.z;
    pts[3].x = pt_min.x; pts[3].y = pt_max.y; pts[3].z = pt_min.z;
    pts[4].x = pt_min.x; pts[4].y = pt_min.y; pts[4].z = pt_max.z;
    pts[5].x = pt_max.x; pts[5].y = pt_min.y; pts[5].z = pt_max.z;
    pts[6].x = pt_max.x; pts[6].y = pt_max.y; pts[6].z = pt_max.z;
    pts[7].x = pt_min.x; pts[7].y = pt_max.y; pts[7].z = pt_max.z;

    viewer.addLine(pts[0], pts[1], r,g,b, id + "_line_01", viewport);
    viewer.addLine(pts[1], pts[2], r,g,b, id + "_line_12", viewport);
    viewer.addLine(pts[2], pts[3], r,g,b, id + "_line_23", viewport);
    viewer.addLine(pts[3], pts[0], r,g,b, id + "_line_30", viewport);
    viewer.addLine(pts[4], pts[5], r,g,b, id + "_line_45", viewport);
    viewer.addLine(pts[5], pts[6], r,g,b, id + "_line_56", viewport);
    viewer.addLine(pts[6], pts[7], r,g,b, id + "_line_67", viewport);
    viewer.addLine(pts[7], pts[4], r,g,b, id + "_line_74", viewport);
    viewer.addLine(pts[0], pts[4], r,g,b, id + "_line_04", viewport);
    viewer.addLine(pts[1], pts[5], r,g,b, id + "_line_15", viewport);
    viewer.addLine(pts[2], pts[6], r,g,b, id + "_line_26", viewport);
    viewer.addLine(pts[3], pts[7], r,g,b, id + "_line_37", viewport);
}

template 
void addCube<PointXYZ>( pcl::visualization::PCLVisualizer &viewer,
                        const PointXYZ &pt_min, const PointXYZ &pt_max,
                        double r, double g, double b, 
                        const std::string &id, int viewport             );

template 
void addCube<PointXYZRGB>( pcl::visualization::PCLVisualizer &viewer,
                        const PointXYZRGB &pt_min, const PointXYZRGB &pt_max,
                        double r, double g, double b, 
                        const std::string &id, int viewport             );


void addPointCloud( pcl::visualization::PCLVisualizer &viewer, 
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                    const std::string &id, int viewport)
{
    PointCloud<PointXYZRGB>::Ptr cloud_rgb(new PointCloud<PointXYZRGB>);
    
    for( size_t p=0; p<cloud->size(); p++ )
    {
        if( (*cloud)[p].z != 0 )
        {
            PointXYZRGB pt;
            pt.x = (*cloud)[p].x;
            pt.y = (*cloud)[p].y;
            pt.z = (*cloud)[p].z;
            pt.r = (*cloud)[p].r;
            pt.g = (*cloud)[p].g;
            pt.b = (*cloud)[p].b;
            cloud_rgb->push_back(pt);
        }

        //if( p%100 > 0 ) continue;

        PointXYZRGB pt;
        pt.x = (*cloud)[p].x + (*cloud)[p].normal_x * 0.01;
        pt.y = (*cloud)[p].y + (*cloud)[p].normal_y * 0.01;
        pt.z = (*cloud)[p].z + (*cloud)[p].normal_z * 0.01;
        
        stringstream ss;
        ss << id << "_line_" << p;
        viewer.addLine((*cloud)[p], pt, 0, 255, 255, ss.str(), viewport);
    }

    viewer.addPointCloud(cloud_rgb, id + "_cloud_rgb", viewport );
}

void addVoxel( pcl::visualization::PCLVisualizer &viewer, 
               const voxel_t &voxel, const float resolution,
               const std::string &id, int viewport, int target, bool only_pos )
{
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    for( int i_x=0; i_x<voxel.size(); i_x++ )
    for( int i_y=0; i_y<voxel[i_x].size(); i_y++ )
    for( int i_z=0; i_z<voxel[i_x][i_y].size(); i_z++ )
    {
        int label = voxel[i_x][i_y][i_z];
        if( label==0 ) continue;
        if( target!=0 && target != label ) continue;
        if( only_pos && (label <= 0 || label == INFINITY) ) continue;
        
        uint8_t r,g,b;
        if( label > 0 )
        {
            r = colors_vis[label%20][0];
            g = colors_vis[label%20][1];
            b = colors_vis[label%20][2];
        }
        else if( label < 0 )
        {
            r = colors_vis[abs(label)%20][0] / 2;
            g = colors_vis[abs(label)%20][1] / 2;
            b = colors_vis[abs(label)%20][2] / 2;
        }
        else if( label == INFINITY )
        {
            r = 0;
            g = 0;
            b = 0;
        }
        else if( label == -INFINITY )
        {
            r = 242;
            g = 242;
            b = 242;
        }        

        PointXYZ pt_min, pt_max;
        pt_min.x = i_x * resolution; pt_max.x = (i_x+1) * resolution;
        pt_min.y = i_y * resolution; pt_max.y = (i_y+1) * resolution; 
        pt_min.z = i_z * resolution; pt_max.z = (i_z+1) * resolution;

        PointXYZRGB pt_ctr;
        pt_ctr.x = (pt_min.x + pt_max.x) * 0.5;
        pt_ctr.y = (pt_min.y + pt_max.y) * 0.5;
        pt_ctr.z = (pt_min.z + pt_max.z) * 0.5;
        pt_ctr.r = r;
        pt_ctr.g = g;
        pt_ctr.b = b;

        //stringstream ss;
        //ss << "[" << i_x << "," << i_y << "," << i_z << "]";
        //addCube(viewer, pt_min, pt_max, r,g,b, ss.str() + id, viewport );
        cloud->push_back(pt_ctr);
    }

    if( voxel.size() )
    {
        viewer.addPointCloud(cloud, id, viewport);

        PointXYZRGB pt_min, pt_max;
        pt_min.x = 0; 
        pt_min.y = 0; 
        pt_min.z = 0; 
        pt_max.x = voxel.size() * resolution;
        pt_max.y = voxel[0].size() * resolution;
        pt_max.z = voxel[0][0].size() * resolution;
        addCube(viewer, pt_min, pt_max, 1,1,1, id + "_bound", viewport);
    }
}

void addTSDF( pcl::visualization::PCLVisualizer &viewer, 
               const tsdf_t &tsdf, const float resolution,
               const std::string &id, int viewport        )
{
    const float tsdf_max =  resolution * 10;
    const float tsdf_min = -resolution * 10;
    const float norm = tsdf_max - tsdf_min;

    vtkSmartPointer<vtkLookupTable> table;
    visualization::getColormapLUT(
        visualization::PCL_VISUALIZER_LUT_BLUE2RED, table );

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    for( int i_x=0; i_x<tsdf.size(); i_x++ )
    for( int i_y=0; i_y<tsdf[i_x].size(); i_y++ )
    for( int i_z=0; i_z<tsdf[i_x][i_y].size(); i_z++ )
    {
        float val = tsdf[i_x][i_y][i_z];
                
        //if( val >= tsdf_max ) continue;
        if( val > 0 ) continue;

        double color[3];
        table->GetColor((val-tsdf_min)/norm , color);
        
        PointXYZ pt_min, pt_max;
        pt_min.x = i_x * resolution; pt_max.x = (i_x+1) * resolution;
        pt_min.y = i_y * resolution; pt_max.y = (i_y+1) * resolution; 
        pt_min.z = i_z * resolution; pt_max.z = (i_z+1) * resolution;

        PointXYZRGB pt_ctr;
        pt_ctr.x = (pt_min.x + pt_max.x) * 0.5;
        pt_ctr.y = (pt_min.y + pt_max.y) * 0.5;
        pt_ctr.z = (pt_min.z + pt_max.z) * 0.5;
        pt_ctr.r = color[0] * 255;
        pt_ctr.g = color[1] * 255;
        pt_ctr.b = color[2] * 255;

        cloud->push_back(pt_ctr);
    }

    viewer.addPointCloud(cloud, id, viewport);

    PointXYZRGB pt_min, pt_max;
    pt_min.x = 0; 
    pt_min.y = 0; 
    pt_min.z = 0; 
    pt_max.x = tsdf.size() * resolution;
    pt_max.y = tsdf[0].size() * resolution;
    pt_max.z = tsdf[0][0].size() * resolution;
    addCube(viewer, pt_min, pt_max, 1,1,1, id + "_bound", viewport);
}

void addWorkspace( pcl::visualization::PCLVisualizer &viewer, 
                   const vector<float> &workspace,
                   double r, double b, double g,
                   const std::string &id, int viewport        )
{
    Eigen::Matrix3f tf_ws_rot
     = Eigen::Quaternionf(workspace[3],
                          workspace[4],
                          workspace[5],
                          workspace[6]).toRotationMatrix();
    Eigen::Matrix4f tf_ws;
    tf_ws << tf_ws_rot(0,0),tf_ws_rot(0,1),tf_ws_rot(0,2),workspace[0],
             tf_ws_rot(1,0),tf_ws_rot(1,1),tf_ws_rot(1,2),workspace[1],
             tf_ws_rot(2,0),tf_ws_rot(2,1),tf_ws_rot(2,2),workspace[2],
                          0,             0,             0,           1;

    PointXYZ pt_min, pt_max;
    pt_min.x = -workspace[7]*0.5;
    pt_max.x =  workspace[7]*0.5;
    pt_min.y = -workspace[8]*0.5;
    pt_max.y =  workspace[8]*0.5;
    pt_min.z = -workspace[9]*0.5;
    pt_max.z =  workspace[9]*0.5;
    
    PointCloud<PointXYZ> cloud;
    cloud.resize(8);
    PointXYZ pts[8];
    cloud[0].x = pt_min.x; cloud[0].y = pt_min.y; cloud[0].z = pt_min.z;
    cloud[1].x = pt_max.x; cloud[1].y = pt_min.y; cloud[1].z = pt_min.z;
    cloud[2].x = pt_max.x; cloud[2].y = pt_max.y; cloud[2].z = pt_min.z;
    cloud[3].x = pt_min.x; cloud[3].y = pt_max.y; cloud[3].z = pt_min.z;
    cloud[4].x = pt_min.x; cloud[4].y = pt_min.y; cloud[4].z = pt_max.z;
    cloud[5].x = pt_max.x; cloud[5].y = pt_min.y; cloud[5].z = pt_max.z;
    cloud[6].x = pt_max.x; cloud[6].y = pt_max.y; cloud[6].z = pt_max.z;
    cloud[7].x = pt_min.x; cloud[7].y = pt_max.y; cloud[7].z = pt_max.z;

    transformPointCloud(cloud,cloud,tf_ws);

    viewer.addLine(cloud[0], cloud[1], r,g,b, id + "_line_01", viewport);
    viewer.addLine(cloud[1], cloud[2], r,g,b, id + "_line_12", viewport);
    viewer.addLine(cloud[2], cloud[3], r,g,b, id + "_line_23", viewport);
    viewer.addLine(cloud[3], cloud[0], r,g,b, id + "_line_30", viewport);
    viewer.addLine(cloud[4], cloud[5], r,g,b, id + "_line_45", viewport);
    viewer.addLine(cloud[5], cloud[6], r,g,b, id + "_line_56", viewport);
    viewer.addLine(cloud[6], cloud[7], r,g,b, id + "_line_67", viewport);
    viewer.addLine(cloud[7], cloud[4], r,g,b, id + "_line_74", viewport);
    viewer.addLine(cloud[0], cloud[4], r,g,b, id + "_line_04", viewport);
    viewer.addLine(cloud[1], cloud[5], r,g,b, id + "_line_15", viewport);
    viewer.addLine(cloud[2], cloud[6], r,g,b, id + "_line_26", viewport);
    viewer.addLine(cloud[3], cloud[7], r,g,b, id + "_line_37", viewport);
}

}
