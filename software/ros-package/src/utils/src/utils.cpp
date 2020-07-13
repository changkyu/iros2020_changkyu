#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>

#include "utils/utils.hpp"

#include <fstream>
#include <tuple>

#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "utils/utils_inline.hpp"
#include "utils/utils_visualization.hpp"

using namespace std;
using namespace pcl;
using namespace cv;

namespace utils
{

#define EIGEN_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406L

uint8_t colors_vis[][3] = 
{
    {230, 25, 75}, // [0] red
    {60, 180, 75}, // [1] green
    {255, 225,25}, // [2] yello
    {0, 130, 200}, // [3] blue
    {245, 130, 48},// [4] orange
    {145, 30, 180},// [5] purple
    {70, 240, 240},
    {240, 50, 230},
    {210, 245, 60},
    {250, 190, 190},
    {0, 128, 128},
    {230, 190, 255},
    {170, 110, 40},
    {255, 250, 200},
    {128, 0, 0},
    {170, 255, 195},
    {128, 128, 0},
    {255, 215, 180},
    {0, 0, 128},
    {128, 128, 128},
    {255, 255, 255}
};

float IntersectionOverUnion(const PolygonMesh &polymesh1,
                            const PolygonMesh &polymesh2, 
                            const float resolution )
{
    PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloudU(new PointCloud<PointXYZRGB>);
    fromPCLPointCloud2(polymesh1.cloud, *cloud1);
    fromPCLPointCloud2(polymesh2.cloud, *cloud2);        
    *cloudU += *cloud1;
    *cloudU += *cloud2;

    // Generate an uniform grid voxels
    PointXYZRGB pt_min, pt_max;
    getMinMax3D(*cloudU, pt_min, pt_max);    
    int n_x = int((pt_max.x - pt_min.x) / resolution + 1);
    int n_y = int((pt_max.y - pt_min.y) / resolution + 1);
    int n_z = int((pt_max.z - pt_min.z) / resolution + 1);
    typename PointCloud<PointXYZRGB>::Ptr cloud_grid(new PointCloud<PointXYZRGB>);
    cloud_grid->points.resize(n_x*n_y*n_z);
    float z=pt_min.z;
    for(int i_z=0; z<=pt_max.z; i_z++)
    {        
        float y=pt_min.y;
        for(int i_y=0; y<=pt_max.y; i_y++)
        {
            float x=pt_min.x;
            for(int i_x=0; x<=pt_max.x; i_x++)
            {
                int i = n_x*n_y*i_z + n_x*i_y + i_x;                
                PointXYZRGB &pt = cloud_grid->points[i];
                pt.x=x; pt.y=y; pt.z=z;                

                x += resolution;
            }
            y += resolution;
        }
        z += resolution;
    }

    vector<int> idxes1, idxes2, idxesA, idxesU;

    CropHull<PointXYZRGB> cropHull;
    cropHull.setDim(3);
    cropHull.setInputCloud(cloud_grid);

    cropHull.setHullCloud(cloud1);
    cropHull.setHullIndices(polymesh1.polygons);
    cropHull.filter(idxes1);
    
    cropHull.setHullCloud(cloud2);
    cropHull.setHullIndices(polymesh2.polygons);
    cropHull.filter(idxes2);

    sort(idxes1.begin(), idxes1.end());
    sort(idxes2.begin(), idxes2.end());    
    set_intersection( idxes1.begin(),idxes1.end(),
                      idxes2.begin(),idxes2.end(),back_inserter(idxesA));
    set_union(        idxes1.begin(),idxes1.end(),
                      idxes2.begin(),idxes2.end(),back_inserter(idxesU));
        
    const float vol_unit = resolution*resolution*resolution;
    float vol1=idxes1.size()*vol_unit,
          vol2=idxes2.size()*vol_unit,
          volA=idxesA.size()*vol_unit,
          volU=idxesU.size()*vol_unit;

#if 0
    cout << "vol1: " << vol1 << ", "
         << "vol2: " << vol2 << ", "
         << "volA: " << volA << ", "
         << "volU: " << volU << ", "
         << "AoU:  " << (volA/volU) << endl;

    typename PointCloud<PointXYZRGB>::Ptr cloud_tmp1(new PointCloud<PointXYZRGB>);
    typename PointCloud<PointXYZRGB>::Ptr cloud_tmp2(new PointCloud<PointXYZRGB>);
    typename PointCloud<PointXYZRGB>::Ptr cloud_tmpA(new PointCloud<PointXYZRGB>);
    typename PointCloud<PointXYZRGB>::Ptr cloud_tmpU(new PointCloud<PointXYZRGB>);
    for( size_t i=0; i<idxes1.size(); i++ )
    {
        cloud_tmp1->push_back(cloud_grid->points[idxes1[i]]);
    }
    for( size_t i=0; i<idxes2.size(); i++ )
    {
        cloud_tmp2->push_back(cloud_grid->points[idxes2[i]]);
    }
    for( size_t i=0; i<idxesA.size(); i++ )
    {
        cloud_tmpA->push_back(cloud_grid->points[idxesA[i]]);
    }
    for( size_t i=0; i<idxesU.size(); i++ )
    {
        cloud_tmpU->push_back(cloud_grid->points[idxesU[i]]);
    }

    PolygonMesh polymeshA, polymeshU;
    ConvexHull<PointXYZRGB> chull;
    chull.setComputeAreaVolume(true);    
    chull.setInputCloud (cloud_tmpA);
    chull.reconstruct(polymeshA);
    chull.setInputCloud (cloud_tmpU);
    chull.reconstruct(polymeshU);

    visualization::PCLVisualizer viewer; 
    int v1, v2, v3, v4, v5, v6;
    viewer.createViewPort (0.0,  0.0, 0.33, 0.5, v1);
    viewer.createViewPort (0.33, 0.0, 0.66, 0.5, v2);
    viewer.createViewPort (0.66, 0.0, 0.99, 0.5, v3);
    viewer.createViewPort (0.0,  0.5, 0.33, 1.0, v4);
    viewer.createViewPort (0.33, 0.5, 0.66, 1.0, v5);
    viewer.createViewPort (0.66, 0.5, 0.99, 1.0, v6);
    
    viewer.setWindowName("AoverU");
    viewer.setSize(600,480);
    viewer.setPosition(0,0);
    viewer.setCameraPosition(-1,0,0,1,0,0,0,0,1);
    viewer.setBackgroundColor (1,1,1);    
    //viewer.addPointCloud(cloud_tmpA,"cloud_tmpA",v1);
    //viewer.addPointCloud(cloud_tmpU,"cloud_tmpU",v2);
    //viewer.addPointCloud(cloud_tmp1,"cloud_tmp1",v3);
    //viewer.addPointCloud(cloud_tmp2,"cloud_tmp2",v4);

    viewer.addPolygonMesh(polymeshA,"polymeshA",v1);
    viewer.addPolygonMesh(polymeshU,"polymeshU",v2);
    viewer.addPolygonMesh(polymesh1,"polymesh1",v3);
    viewer.addPolygonMesh(polymesh2,"polymesh2",v4);
    viewer.addPolygonMesh(polymesh1,"polymesh11",v5);
    viewer.addPolygonMesh(polymesh2,"polymesh22",v5);
    
    viewer.setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_COLOR, 
        colors_vis[5][0]/255.,colors_vis[5][1]/255.,colors_vis[5][2]/255., 
        "polymeshA", v1);
    viewer.setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_COLOR, 
        colors_vis[2][0]/255.,colors_vis[2][1]/255.,colors_vis[2][2]/255., 
        "polymeshU", v2);
    viewer.setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_COLOR, 
        colors_vis[0][0]/255.,colors_vis[0][1]/255.,colors_vis[0][2]/255., 
        "polymesh1", v3);
    viewer.setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_COLOR, 
        colors_vis[3][0]/255.,colors_vis[3][1]/255.,colors_vis[3][2]/255., 
        "polymesh2", v4);
    viewer.setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_COLOR, 
        colors_vis[0][0]/255.,colors_vis[0][1]/255.,colors_vis[0][2]/255., 
        "polymesh11", v5);
    viewer.setPointCloudRenderingProperties( visualization::PCL_VISUALIZER_COLOR, 
        colors_vis[3][0]/255.,colors_vis[3][1]/255.,colors_vis[3][2]/255., 
        "polymesh22", v5);


/*
    viewer.addText3D("A", cloud_tmpA->points[0], 0.01, 0,1,0, "name_A", v1);
    viewer.addText3D("U", cloud_tmpU->points[0], 0.01, 1,0,1, "name_U", v2);
    viewer.addText3D("1", cloud_tmp1->points[0], 0.01, 1,0,0, "name_1", v3);
    viewer.addText3D("2", cloud_tmp2->points[0], 0.01, 0,0,1, "name_2", v4);
*/
    viewer.spin();
#endif

    return volA / volU;
}

template<typename PointT>
void PolygonMesh2PointCloud_helper( const PointT &ptA, 
                                    const PointT &ptB, 
                                    const PointT &ptC, 
                                    PointCloud<PointT> &cloud, 
                                    const float resolution=0.005 )
{
    float distAB = compute_dist(ptA, ptB);
    float distBC = compute_dist(ptB, ptC);
    float distCA = compute_dist(ptC, ptA);

    float distMax;
    PointT ptNew;
    if( distAB > distBC )
    {
        if( distAB > distCA )
        {
            distMax = distAB;
            ptNew.x = (ptA.x + ptB.x) * 0.5;
            ptNew.y = (ptA.y + ptB.y) * 0.5;
            ptNew.z = (ptA.z + ptB.z) * 0.5;
            if( distMax > resolution )
            {
                PolygonMesh2PointCloud_helper(ptNew,ptB,ptC,cloud,resolution);
                PolygonMesh2PointCloud_helper(ptA,ptNew,ptC,cloud,resolution);
            }
        }
        else
        {
            distMax = distCA;
            ptNew.x = (ptC.x + ptA.x) * 0.5;
            ptNew.y = (ptC.y + ptA.y) * 0.5;
            ptNew.z = (ptC.z + ptA.z) * 0.5;
            if( distMax > resolution )
            {
                PolygonMesh2PointCloud_helper(ptNew,ptB,ptC,cloud,resolution);
                PolygonMesh2PointCloud_helper(ptA,ptB,ptNew,cloud,resolution);
            }
        }
    }
    else
    {
        if( distBC > distCA )
        {
            distMax = distBC;
            ptNew.x = (ptB.x + ptC.x) * 0.5;
            ptNew.y = (ptB.y + ptC.y) * 0.5;
            ptNew.z = (ptB.z + ptC.z) * 0.5;
            if( distMax > resolution )
            {
                PolygonMesh2PointCloud_helper(ptA,ptNew,ptC,cloud,resolution);
                PolygonMesh2PointCloud_helper(ptA,ptB,ptNew,cloud,resolution);
            }
        }
        else
        {
            distMax = distCA;
            ptNew.x = (ptC.x + ptA.x) * 0.5;
            ptNew.y = (ptC.y + ptA.y) * 0.5;
            ptNew.z = (ptC.z + ptA.z) * 0.5;
            if( distMax > resolution )
            {
                PolygonMesh2PointCloud_helper(ptNew,ptB,ptC,cloud,resolution);
                PolygonMesh2PointCloud_helper(ptA,ptB,ptNew,cloud,resolution);
            }
        }
    }
    cloud.push_back(ptNew);
}

template<typename PointT>
void PolygonMesh2PointCloud(const PolygonMesh &polymesh, 
                            PointCloud<PointT> &cloud,
                            const float resolution=0.005)
{    
    PointCloud<PointT> points;
    fromPCLPointCloud2(polymesh.cloud, points);    
    cloud.clear();
    copyPointCloud(points, cloud);    
    for( size_t i=0; i<polymesh.polygons.size(); i++ )
    {
        const Vertices &polygon = polymesh.polygons[i];        
        PointT &ptA = points[polygon.vertices[0]];
        PointT &ptB = points[polygon.vertices[1]];
        PointT &ptC = points[polygon.vertices[2]];        

        PolygonMesh2PointCloud_helper<PointT>(ptA,ptB,ptC, cloud, resolution);
    }
}

template void PolygonMesh2PointCloud<PointXYZ>( const PolygonMesh &polymesh, 
                                                PointCloud<PointXYZ> &cloud, 
                                                const float resolution=0.005  );
template void PolygonMesh2PointCloud<PointXYZRGB>( const PolygonMesh &polymesh, 
                                                PointCloud<PointXYZRGB> &cloud,
                                                const float resolution=0.005  );

template<typename PointT>
void PointClouds2LabelImage(
    const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
    cv::Mat &image, const size_t width, const size_t height,
    const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT,
    const std::vector<int> &labels_in )
{
    vector<int> labels;
    if( labels_in.size()==0 )
    {
        for( size_t o=0; o<clouds.size(); o++ )
        {
            labels.push_back(o+1);
        }
    }
    else
    {
        labels = labels_in;
    }

    Eigen::Matrix4f world2cam = cam_RT.inverse();

    image = cv::Mat::zeros(height, width, CV_8U);
    Mat depth = cv::Mat::zeros(height, width, CV_32F);
    for( size_t o=0; o<clouds.size(); o++ )
    {
        PointCloud<PointT> cloud_cam;
        transformPointCloud(*clouds[o], cloud_cam, world2cam);

        for( size_t p=0; p<cloud_cam.size(); p++)
        {
            PointT &pt = cloud_cam[p];

            float x = pt.x/pt.z;
            float y = pt.y/pt.z;
            //float dep = pt.z * 1000.0f;

            int c = (int)(x*cam_K(0,0) + y*cam_K(0,1) + cam_K(0,2) + 0.5);
            int r = (int)(x*cam_K(1,0) + y*cam_K(1,1) + cam_K(1,2) + 0.5);
            
            if( (0 <= c) && (c < width) && (0 <= r) && (r < height) )
            {
                float &dep = depth.at<float>(r,c);
                if( dep==0 || dep > pt.z )
                {
                    image.at<uint8_t>(r,c) = (uint8_t)labels[o];
                    dep = pt.z;
                }
            }
        }
    }
}

template
void PointClouds2LabelImage<PointXYZ>(
    const std::vector<typename pcl::PointCloud<PointXYZ>::Ptr> &clouds,
    cv::Mat &image, const size_t width, const size_t height,
    const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT,
    const std::vector<int> &labels_in );

template
void PointClouds2LabelImage<PointXYZRGB>(
    const std::vector<typename pcl::PointCloud<PointXYZRGB>::Ptr> &clouds,
    cv::Mat &image, const size_t width, const size_t height,
    const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT,
    const std::vector<int> &labels_in );

template<typename PointT>
void PointCloud2Depth( const PointCloud<PointT> &cloud,
                       cv::Mat &depth, const size_t width, const size_t height,
                       const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT)
{
    Eigen::Matrix4f world2cam = cam_RT.inverse();

    PointCloud<PointT> cloud_cam;
    transformPointCloud(cloud, cloud_cam, world2cam);

    depth = cv::Mat::zeros(height, width, CV_16U);    
    for( size_t p=0; p<cloud_cam.size(); p++)
    {
        PointT &pt = cloud_cam[p];

        float x = pt.x/pt.z;
        float y = pt.y/pt.z;
        float dep = pt.z * 1000.0f;

        int c = (int)(x*cam_K(0,0) + y*cam_K(0,1) + cam_K(0,2) + 0.5);
        int r = (int)(x*cam_K(1,0) + y*cam_K(1,1) + cam_K(1,2) + 0.5);
        
        if( (0 <= c) && (c < width) && (0 <= r) && (r < height) )
        {
            uint16_t &val = depth.at<uint16_t>(r,c);            
            if( val==0 || val>dep ) val = dep;
        }
    }
}

template
void PointCloud2Depth( const PointCloud<PointXYZRGB> &cloud,
                       cv::Mat &depth, const size_t width, const size_t height,
                       const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT);

template<typename PointT>
static void ClearZeroZ( PointCloud<PointT> &cloud )
{
    PointCloud<PointT> cloud_tmp;
    for( int p=0; p<cloud.size(); p++ )
    {
        PointT &pt = cloud[p];
        if( pt.z == 0 ) continue;

        cloud_tmp.push_back(pt);
    }

    cloud.clear();
    copyPointCloud(cloud_tmp, cloud);
}

template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized            )
{
    int rows = depth.rows;
    int cols = depth.cols;
    cloud.width  = cols;
    cloud.height = rows;
    cloud.points.resize(cols*rows);

    for( int r=0; r<rows; r++ )
    for( int c=0; c<cols; c++ )
    {
        PointT &pt = cloud(c,r);

        // mask out
        if( mask.rows > 0 && mask.cols > 0 && mask.at<uint8_t>(r,c) == 0 )
        {
            pt.x = 0; pt.y = 0; pt.z = 0;
            continue;
        } 

        // 2D -> 3D (camera coordinate)
        float z = (float)depth.at<TYPE_DEPTH>(r,c) * depth_scale;
        if( z < 0.10 )
        {
            pt.x = 0; pt.y = 0; pt.z = 0;            
        }
        float x = (((float)c) - cam_in[2]) / cam_in[0] * z;
        float y = (((float)r) - cam_in[5]) / cam_in[4] * z;

        pt.x = x; pt.y = y; pt.z = z;
    }

    if( !isOrganized )    
    {        
        ClearZeroZ(cloud);
        cloud.width = cloud.size();
        cloud.height = 1;
    }

    // camera -> world coordinate    
    if( cam_ex.size() == 16 )
    {
        Eigen::Matrix4f tf;
        tf << cam_ex[0],  cam_ex[1],  cam_ex[2],  cam_ex[3],
              cam_ex[4],  cam_ex[5],  cam_ex[6],  cam_ex[7],
              cam_ex[8],  cam_ex[9],  cam_ex[10], cam_ex[11],
              cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15];
        transformPointCloud(cloud, cloud, tf);
    }
}

template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,                          
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized            )
{
    vector<float> cam_ex_id(16);
    cam_ex_id[0] =1; cam_ex_id[1] =0; cam_ex_id[2] =0; cam_ex_id[3] =0; 
    cam_ex_id[4] =0; cam_ex_id[5] =1; cam_ex_id[6] =0; cam_ex_id[7] =0; 
    cam_ex_id[8] =0; cam_ex_id[9] =0; cam_ex_id[10]=1; cam_ex_id[11]=0; 
    cam_ex_id[12]=0; cam_ex_id[13]=0; cam_ex_id[14]=0; cam_ex_id[15]=1;

    PointCloudfromDepth<PointT,TYPE_DEPTH>(
        cloud, depth, depth_scale, cam_in, cam_ex_id, mask, true);

    int rows = depth.rows;
    int cols = depth.cols;
    for( int r=0; r<rows; r++ )
    for( int c=0; c<cols; c++ )
    {
        PointT &pt = cloud(c,r);

        pt.r = image.at<cv::Vec3b>(r,c)[0];
        pt.g = image.at<cv::Vec3b>(r,c)[1];
        pt.b = image.at<cv::Vec3b>(r,c)[2];
    }

    if( !isOrganized )    
    {        
        ClearZeroZ(cloud);
        cloud.width = cloud.size();
        cloud.height = 1;
    }

    // camera -> world coordinate
    if( cam_ex.size() == 16 )
    {
        Eigen::Matrix4f tf;
        tf << cam_ex[0],  cam_ex[1],  cam_ex[2],  cam_ex[3],
              cam_ex[4],  cam_ex[5],  cam_ex[6],  cam_ex[7],
              cam_ex[8],  cam_ex[9],  cam_ex[10], cam_ex[11],
              cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15];
        transformPointCloud(cloud, cloud, tf);
    }    
}

template<typename PointT, typename TYPE_DEPTH>
void PointCloudsfromDepth( map<uint8_t,typename PointCloud<PointT>::Ptr> &clouds,
                           const cv::Mat &seg,
                           const cv::Mat &depth,
                           const float depth_scale,
                           const std::vector<float> &cam_in,
                           const std::vector<float> &cam_ex,                          
                           const cv::Mat &image,
                           const cv::Mat &mask              )
{
    vector<float> cam_ex_id(16);
    cam_ex_id[0] =1; cam_ex_id[1] =0; cam_ex_id[2] =0; cam_ex_id[3] =0; 
    cam_ex_id[4] =0; cam_ex_id[5] =1; cam_ex_id[6] =0; cam_ex_id[7] =0; 
    cam_ex_id[8] =0; cam_ex_id[9] =0; cam_ex_id[10]=1; cam_ex_id[11]=0; 
    cam_ex_id[12]=0; cam_ex_id[13]=0; cam_ex_id[14]=0; cam_ex_id[15]=1;

    clouds.clear();

    PointCloud<PointT> cloud;
    PointCloudfromDepth<PointT,TYPE_DEPTH>(
        cloud, depth, depth_scale, cam_in, cam_ex_id, mask, true);

    int rows = depth.rows;
    int cols = depth.cols;
    for( int r=0; r<rows; r++ )
    for( int c=0; c<cols; c++ )
    {
        PointT &pt = cloud(c,r);

        pt.b = image.at<cv::Vec3b>(r,c)[0];
        pt.g = image.at<cv::Vec3b>(r,c)[1];
        pt.r = image.at<cv::Vec3b>(r,c)[2];

        uint8_t label = seg.at<uint8_t>(r,c);

        typename PointCloud<PointT>::Ptr cloud_seg;
        typename map< uint8_t,
                      typename PointCloud<PointT>::Ptr >::iterator it_cloud
         = clouds.find(label);
        if( it_cloud == clouds.end() )
        {
            cloud_seg
             = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);
            clouds.insert(
              pair<uint8_t,typename PointCloud<PointT>::Ptr>(label, cloud_seg));
        }
        else
        {
            cloud_seg = it_cloud->second;
        }
        cloud_seg->push_back(pt);
    }

    Eigen::Matrix4f tf;
    if( cam_ex.size()==0 )
    {
        tf = Eigen::Matrix4f::Identity();
    }
    else
    {
        tf << cam_ex[0],  cam_ex[1],  cam_ex[2],  cam_ex[3],
              cam_ex[4],  cam_ex[5],  cam_ex[6],  cam_ex[7],
              cam_ex[8],  cam_ex[9],  cam_ex[10], cam_ex[11],
              cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15];
    }

    for( typename map<uint8_t,typename PointCloud<PointT>::Ptr>::iterator 
         it=clouds.begin(); it!=clouds.end(); it++ )
    {        
        ClearZeroZ(*it->second);

        // camera -> world coordinate    
        transformPointCloud(*it->second, *it->second, tf);
    }    
}

template
void PointCloudfromDepth<PointXYZ, uint16_t>( 
                          PointCloud<PointXYZ> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized               );

template 
void PointCloudfromDepth<PointXYZRGB, uint16_t>( 
                          PointCloud<PointXYZRGB> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized               );

template 
void PointCloudfromDepth<PointXYZRGBNormal, uint16_t>( 
                          PointCloud<PointXYZRGBNormal> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized               );

template 
void PointCloudfromDepth<PointXYZRGB, uint16_t>( 
                          PointCloud<PointXYZRGB> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized              );

template 
void PointCloudfromDepth<PointXYZRGBNormal, uint16_t>( 
                          PointCloud<PointXYZRGBNormal> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized              );

template
void PointCloudsfromDepth<PointXYZRGB, uint16_t>( 
                           map<uint8_t,PointCloud<PointXYZRGB>::Ptr> &clouds,
                           const cv::Mat &seg,
                           const cv::Mat &depth,
                           const float depth_scale,
                           const std::vector<float> &cam_in,
                           const std::vector<float> &cam_ex,                          
                           const cv::Mat &image,
                           const cv::Mat &mask              );

void PolygonMesh2Depth(const PolygonMesh &polymesh, 
                       cv::Mat &depth, const size_t width, const size_t height,
                       const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT,
                       const float resolution)
{
    PointCloud<PointXYZ> cloud;
    PolygonMesh2PointCloud(polymesh, cloud, resolution);
    PointCloud2Depth(cloud, depth, width, height, cam_K, cam_RT);
}

template<typename PointT>
void PointCloud2Depth( const PointCloud<PointT> &cloud,
                       cv::Mat &depth, const size_t width, const size_t height,
                       const std::vector<float> &cam_K,
                       const std::vector<float> &cam_RT             )
{
    assert(cam_K.size()==9 && "cam_K.size()==9");
    assert(cam_RT.size()==16 && "cam_RT.size()==16");

    Eigen::Matrix3f K;
    K << cam_K[0], cam_K[1], cam_K[2],
         cam_K[3], cam_K[4], cam_K[5],
         cam_K[6], cam_K[7], cam_K[8];

    Eigen::Matrix4f RT;
    RT << cam_RT[0], cam_RT[1], cam_RT[2],  cam_RT[3],
          cam_RT[4], cam_RT[5], cam_RT[6],  cam_RT[7],
          cam_RT[8], cam_RT[9], cam_RT[10], cam_RT[11];

    PointCloud2Depth(cloud, depth, width, height, K, RT);
}

void PolygonMesh2Depth(const PolygonMesh &polymesh, 
                       cv::Mat &depth, const size_t width, const size_t height,
                       const std::vector<float> &cam_K,
                       const std::vector<float> &cam_RT,
                       const float resolution             )
{
    assert(cam_K.size()==9 && "cam_K.size()==9");
    assert(cam_RT.size()==16 && "cam_RT.size()==16");

    Eigen::Matrix3f K;
    K << cam_K[0], cam_K[1], cam_K[2],
         cam_K[3], cam_K[4], cam_K[5],
         cam_K[6], cam_K[7], cam_K[8];

    Eigen::Matrix4f RT;
    RT << cam_RT[0],  cam_RT[1],  cam_RT[2],  cam_RT[3],
          cam_RT[4],  cam_RT[5],  cam_RT[6],  cam_RT[7],
          cam_RT[8],  cam_RT[9],  cam_RT[10], cam_RT[11];
          cam_RT[12], cam_RT[13], cam_RT[14], cam_RT[14];

    PolygonMesh2Depth(polymesh, depth, width, height, K, RT, resolution);
}

template<typename PointT>
float compute_dist(PointT &pt1, PointT &pt2)
{
    return sqrt( (pt1.x-pt2.x)*(pt1.x-pt2.x)+
                 (pt1.y-pt2.y)*(pt1.y-pt2.y)+
                 (pt1.z-pt2.z)*(pt1.z-pt2.z)  );
}

template float compute_dist<PointXYZ>(PointXYZ &pt1, PointXYZ &pt2);
template float compute_dist<PointXYZRGB>(PointXYZRGB &pt1, PointXYZRGB &pt2);

template<typename PointT>
void FillPolygonMesh( const PolygonMesh &polymesh, 
                      PointCloud<PointT> &cloud_filled, 
                      const float resolution            )
{
    typename PointCloud<PointT>::Ptr cloud_hull(new PointCloud<PointT>);
    fromPCLPointCloud2(polymesh.cloud, *cloud_hull);
    
    PointT pt_min, pt_max;
    getMinMax3D(*cloud_hull, pt_min, pt_max);
    
    typename PointCloud<PointT>::Ptr cloud_box(new PointCloud<PointT>);    
    int n_x = ceil((pt_max.x - pt_min.x) / resolution) + 2;
    int n_y = ceil((pt_max.y - pt_min.y) / resolution) + 2;
    int n_z = ceil((pt_max.z - pt_min.z) / resolution) + 2;
    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {
        PointT pt;
        pt.x = i_x*resolution + pt_min.x - resolution;
        pt.y = i_y*resolution + pt_min.y - resolution;
        pt.z = i_z*resolution + pt_min.z - resolution;

        cloud_box->push_back(pt);
    }

    // Note crophull is buggy
    CropHull<PointT> cropHull;
    cropHull.setDim(3);    
    cropHull.setInputCloud(cloud_box);
    cropHull.setHullCloud(cloud_hull);
    cropHull.setHullIndices(polymesh.polygons);    
    cropHull.filter(cloud_filled);
}

template 
void FillPolygonMesh<PointXYZ>( const PolygonMesh &polymesh_in, 
                                PointCloud<PointXYZ> &cloud_filled, 
                                const float resolution             );
template 
void FillPolygonMesh<PointXYZRGB>( const PolygonMesh &polymesh_in, 
                                   PointCloud<PointXYZRGB> &cloud_filled, 
                                   const float resolution               );

template<typename Scalar>
Eigen::Quaternion<Scalar> UnitRandom_Quaternion()
{    
    const Scalar u1 = Eigen::internal::random<Scalar>(0, 1),
    u2 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI),
    u3 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI);
    const Scalar a = sqrt(1 - u1),
    b = sqrt(u1);
    return Eigen::Quaternion<Scalar>(a * sin(u2), 
                                     a * cos(u2), 
                                     b * sin(u3), 
                                     b * cos(u3));
}

void UnitRandom_CameraExtrinsic( Eigen::Matrix4f &cam_extrinsic,
                                 const std::pair<float,float> d_range,
                                 const std::pair<float,float> a_range,
                                 const std::pair<float,float> e_range  )
{
    float a = Eigen::internal::random<float>(a_range.first, a_range.second);
    float e = Eigen::internal::random<float>(e_range.first, e_range.second);
    float d = Eigen::internal::random<float>(d_range.first, d_range.second);

    Eigen::Matrix4f world2cam;
    world2cam << 1,0,0,0,
                 0,1,0,0,
                 0,0,1,d,
                 0,0,0,1;

    Eigen::Matrix3f m1;
    m1 =  Eigen::AngleAxisf( e, Eigen::Vector3f::UnitY())
         *Eigen::AngleAxisf(-a, Eigen::Vector3f::UnitZ());

    Eigen::Matrix3f m2;
    m2 =  Eigen::AngleAxisf(M_PI*0.5, Eigen::Vector3f::UnitY())
         *Eigen::AngleAxisf(M_PI*0.5, Eigen::Vector3f::UnitX());

    world2cam.block(0,0,3,3) = m2 * m1;

    cam_extrinsic = world2cam.inverse();
}

void UnitRandom_NoDup( const int min, const int max, const size_t n_output, 
                       std::vector<int> &numbers_out )
{
    assert(n_output>0 && "UnitRandom_NoDup: n_output > 0");
    assert(max >= min && "UnitRandom_NoDup: max >= min");
    assert((max-min) >= (n_output-1) && "UnitRandom_NoDup: max-min > n_output-1");

    map<int,int> map_replace;
    vector<int> idxes;
    size_t size = (size_t)(max-min)+1;
    for( size_t i=0; i<n_output; i++ )
    {
        int offset=0;
        int idx_rnd = rand() % size;

        map<int,int>::iterator it_map = map_replace.find(idx_rnd);
        if( it_map != map_replace.end() )
        {
            numbers_out.push_back(min + it_map->second);

            map<int,int>::iterator it_map2 = map_replace.find(size-1);
            if( it_map2 != map_replace.end() ) it_map->second = it_map2->second;
            else                               it_map->second = size-1;
        }
        else
        {
            numbers_out.push_back(min + idx_rnd);

            map<int,int>::iterator it_map2 = map_replace.find(size-1);
            if( it_map2 != map_replace.end() ) 
                map_replace.insert(pair<int,int>(idx_rnd,it_map2->second));
            else
                map_replace.insert(pair<int,int>(idx_rnd,size-1));
        }

        size--;
    }
}

template Eigen::Quaternion<float> UnitRandom_Quaternion<float>();
template Eigen::Quaternion<double> UnitRandom_Quaternion<double>();

template<typename PointT>
void PointClouds2Voxel( const vector<typename PointCloud<PointT>::Ptr> &clouds,
                        voxel_t &voxels, PointT &voxel_origin,
                        int &n_x, int &n_y, int &n_z,
                        const float resolution,
                        const vector<int> labels_in                         )
{
    const float resolution_half = resolution*0.5;

    size_t n_objs = clouds.size();
    PointCloud<PointT> cloud_all;
    for( size_t o=0; o<n_objs; o++ )
    {
        cloud_all += *clouds[o];
    }
    
    vector<int> labels;
    if( labels_in.size()==0 )
    {
        for( size_t o=0; o<n_objs; o++ )
        {
            labels.push_back(o+1);
        }
    }
    else
    {
        labels = labels_in;
    }

    // initialize voxel
    PointT pt_min, pt_max;
    getMinMax3D(cloud_all, pt_min, pt_max); 
    pt_min.x -= resolution_half;   
    pt_min.y -= resolution_half;
    pt_min.z -= resolution_half;
    pt_max.x += resolution_half;   
    pt_max.y += resolution_half;
    pt_max.z += resolution_half;
    if( n_x <= 0 || n_y <= 0 || n_z <= 0 )
    {
        n_x = ceil((pt_max.x - pt_min.x)/resolution);
        n_y = ceil((pt_max.y - pt_min.y)/resolution);
        n_z = ceil((pt_max.z - pt_min.z)/resolution);

        if( voxel_origin.x==INFINITY || voxel_origin.x==-INFINITY ||
            voxel_origin.y==INFINITY || voxel_origin.y==-INFINITY ||
            voxel_origin.z==INFINITY || voxel_origin.z==-INFINITY    )
        {
            voxel_origin.x = pt_min.x;
            voxel_origin.y = pt_min.y;
            voxel_origin.z = pt_min.z;
        }
    }
    else
    {
        PointT pt_ctr;
        pt_ctr.x = (pt_min.x + pt_max.x) * 0.5;
        pt_ctr.y = (pt_min.y + pt_max.y) * 0.5;
        pt_ctr.z = (pt_min.z + pt_max.z) * 0.5;

        if( voxel_origin.x==INFINITY || voxel_origin.x==-INFINITY ||
            voxel_origin.y==INFINITY || voxel_origin.y==-INFINITY ||
            voxel_origin.z==INFINITY || voxel_origin.z==-INFINITY    )
        {
            voxel_origin.x = pt_ctr.x - (((float)n_x) * resolution_half);
            voxel_origin.y = pt_ctr.y - (((float)n_y) * resolution_half);
            //voxel_origin.z = pt_ctr.z - (((float)n_z) * resolution_half);
            voxel_origin.z = pt_min.z;
        }
    }

    voxels.resize(n_x);
    for( int i_x=0; i_x<n_x; i_x++ )
    {
        voxels[i_x].resize(n_y);
        for( int i_y=0; i_y<n_y; i_y++ )
        {
            voxels[i_x][i_y].resize(n_z);
            for( int i_z=0; i_z<n_z; i_z++ )
            {
                voxels[i_x][i_y][i_z] = 0;
            }
        }    
    }

    // fill the voxel
    for( size_t o=0; o<clouds.size(); o++ )
    {
        typename PointCloud<PointT>::Ptr cloud = clouds[o];
        for( size_t p=0; p<cloud->size(); p++ )
        {
            PointT &pt = cloud->points[p];            

            int i_x = (pt.x - voxel_origin.x) / resolution;
            int i_y = (pt.y - voxel_origin.y) / resolution;
            int i_z = (pt.z - voxel_origin.z) / resolution;

            if( i_x < 0 || n_x <= i_x || 
                i_y < 0 || n_y <= i_y || 
                i_z < 0 || n_z <= i_z    ) continue;

            voxels[i_x][i_y][i_z] = labels[o];
        }
    }
}

template<typename PointT>
void PointCloud2Voxel( const PointCloud<PointT> &cloud,
                       voxel_t &voxels, PointT &voxel_origin,
                       int &n_x, int &n_y, int &n_z,
                       const float resolution,
                       const int label                        )
{
    typename PointCloud<PointT>::Ptr cloud_ptr(new PointCloud<PointT>);
    *cloud_ptr += cloud;

    vector<typename PointCloud<PointT>::Ptr> clouds(1);
    clouds[0] = cloud_ptr;
    vector<int> labels(1);
    labels[0] = label;

    PointClouds2Voxel( clouds, voxels, 
                       voxel_origin, n_x,n_y,n_z, resolution, labels);
}

template
void PointClouds2Voxel<PointXYZ>( 
                       const vector<typename PointCloud<PointXYZ>::Ptr> &clouds,
                       voxel_t &voxels, PointXYZ &voxel_origin,
                       int &n_x, int &n_y, int &n_z,
                       const float resolution,
                       const vector<int> labels_in                         );

template
void PointClouds2Voxel<PointXYZRGB>( 
                       const vector<typename PointCloud<PointXYZRGB>::Ptr> &clouds,
                       voxel_t &voxels, PointXYZRGB &voxel_origin,
                       int &n_x, int &n_y, int &n_z,
                       const float resolution,
                       const vector<int> labels_in                         );

template
void PointCloud2Voxel<PointXYZ>( 
                       const PointCloud<PointXYZ> &cloud,
                       voxel_t &voxels, PointXYZ &voxel_origin,
                       int &n_x, int &n_y, int &n_z,
                       const float resolution,
                       const int label                        );

template
void PointCloud2Voxel<PointXYZRGB>( 
                       const PointCloud<PointXYZRGB> &cloud,
                       voxel_t &voxels, PointXYZRGB &voxel_origin,
                       int &n_x, int &n_y, int &n_z,
                       const float resolution,
                       const int label                        );

template<typename PointT>
void PointCloud2PointCloudVox( const PointCloud<PointT> &cloud_in,
                               PointCloud<PointT> &cloud_vox,
                               const float resolution             )
{
    voxel_t voxel;
    PointT pt_origin;
    pt_origin.x = INFINITY;
    pt_origin.y = INFINITY;
    pt_origin.z = INFINITY;
    int n_x=-1, n_y=-1, n_z=-1;
    PointCloud2Voxel<PointT>(cloud_in, voxel, pt_origin, n_x, n_y, n_z, resolution, 1);

    PointCloud<PointXYZRGB> cloud_rgb;
    Voxel2PointCloud( voxel, cloud_rgb, resolution, 0, 1);

    cloud_vox.resize(cloud_rgb.size());
    for( int p=0; p<cloud_rgb.size(); p++ )
    {
        cloud_vox[p].x = cloud_rgb[p].x + pt_origin.x;
        cloud_vox[p].y = cloud_rgb[p].y + pt_origin.y;
        cloud_vox[p].z = cloud_rgb[p].z + pt_origin.z;
    }
}

template
void PointCloud2PointCloudVox<PointXYZRGB>( 
                               const PointCloud<PointXYZRGB> &cloud_in,
                               PointCloud<PointXYZRGB> &cloud_vox,
                               const float resolution             );

template<typename PointT>
void Voxel2PointClouds( const voxel_t &voxel, 
                        map<uint8_t,typename PointCloud<PointT>::Ptr> &clouds,
                        const float resolution )
{
    assert(voxel.size()>0       && "voxel.size() > 0"       );
    assert(voxel[0].size()>0    && "voxel[0].size() > 0"    );
    assert(voxel[0][0].size()>0 && "voxel[0][0[].size() > 0");

    clouds.clear();

    int n_x = voxel.size();
    int n_y = voxel[0].size();
    int n_z = voxel[0][0].size();

    const float resolution_half = resolution * 0.5;

    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {
        int label = voxel[i_x][i_y][i_z];
        if( label > 0 && 
            label != (int) INFINITY && 
            label != (int)-INFINITY    )
        {
            PointT pt;
            pt.x = i_x * resolution + resolution_half;
            pt.y = i_y * resolution + resolution_half;
            pt.z = i_z * resolution + resolution_half;

            typename map<uint8_t,typename PointCloud<PointT>::Ptr >::iterator it
             = clouds.find((uint8_t)label);
            typename PointCloud<PointT>::Ptr cloud;
            if( it == clouds.end() )
            {
                cloud = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);
                clouds.insert(
                  pair<uint8_t,typename PointCloud<PointT>::Ptr>(label,cloud) );
            }
            else
            {
                cloud = it->second;
            }
            cloud->push_back(pt);            
        }
    }
}

template
void Voxel2PointClouds<PointXYZ>( 
                       const voxel_t &voxel, 
                       map<uint8_t,PointCloud<PointXYZ>::Ptr> &clouds,
                       const float resolution );

template
void Voxel2PointClouds<PointXYZRGB>( 
                       const voxel_t &voxel, 
                       map<uint8_t,PointCloud<PointXYZRGB>::Ptr> &clouds,
                       const float resolution );

template <typename Scalar>
void Voxel2PointCloud( const vector<vector<vector<Scalar> > > &voxel, 
                       PointCloud<PointXYZRGB> &cloud, 
                       const float size, 
                       const float val_min, const float val_max )
{
    assert(voxel.size()>0       && "voxel.size() > 0"       );
    assert(voxel[0].size()>0    && "voxel[0].size() > 0"    );
    assert(voxel[0][0].size()>0 && "voxel[0][0[].size() > 0");
    
    int n_x = voxel.size();
    int n_y = voxel[0].size();
    int n_z = voxel[0][0].size();

    const float size_half = size * 0.5;

    vtkSmartPointer<vtkLookupTable> table;
    visualization::getColormapLUT(
        visualization::PCL_VISUALIZER_LUT_BLUE2RED, table );

    float min = val_min;
    float max = val_max;
    float norm = 0;

    bool isLabel = false;
    if( is_same<Scalar,bool>::value  ||
        is_same<Scalar,char>::value  ||
        is_same<Scalar,short>::value ||
        is_same<Scalar,int>::value      )
    {
        isLabel = true;
    }
    else
    {
        if( min==INFINITY || max==-INFINITY )
        {
            for( int i_x=0; i_x<n_x; i_x++ )
            for( int i_y=0; i_y<n_y; i_y++ )
            for( int i_z=0; i_z<n_z; i_z++ )
            {
                double val = voxel[i_x][i_y][i_z];
                if( val == val ) // check nan
                {
                    if( min > val ) min = val;
                    if( max < val ) max = val;
                }
            }
        }
        float norm = max - min;
    }

    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {
        Scalar val = voxel[i_x][i_y][i_z];        
        if( val != 0 )
        {
            PointXYZRGB pt;
            pt.x = i_x * size + size_half;
            pt.y = i_y * size + size_half;
            pt.z = i_z * size + size_half;

            double color[3];
            if( isLabel )
            {
                color[0] = colors_vis[(int)val%21][0] / 255.0f;
                color[1] = colors_vis[(int)val%21][1] / 255.0f;
                color[2] = colors_vis[(int)val%21][2] / 255.0f;
            }
            else
            {
                table->GetColor((val-min)/norm , color);
            }

            pt.r = color[0] * 255;
            pt.g = color[1] * 255;
            pt.b = color[2] * 255;

            cloud.push_back(pt);            
        }
    }
}

template 
void Voxel2PointCloud<bool> ( const vector<vector<vector<bool> > > &voxel, 
                              PointCloud<PointXYZRGB> &cloud, 
                              const float size, 
                              const float val_min, const float val_max );
template 
void Voxel2PointCloud<int>  ( const vector<vector<vector<int> > > &voxel, 
                              PointCloud<PointXYZRGB> &cloud, 
                              const float size, 
                              const float val_min, const float val_max );
template 
void Voxel2PointCloud<float>( const vector<vector<vector<float> > > &voxel, 
                              PointCloud<PointXYZRGB> &cloud, 
                              const float size, 
                              const float val_min, const float val_max );

template<typename PointT, typename TYPE_DEPTH>
void VoxelsfromDepth( voxel_t &voxel, 
                      PointT &voxel_origin,
                      int &n_x, int &n_y, int &n_z,
                      const float resolution,                      
                      map<uint8_t,typename PointCloud<PointT>::Ptr> &seg2cloud,
                      map<uint8_t,typename PointCloud<PointT>::Ptr> &seg2hidden,
                      const cv::Mat &seg,
                      const cv::Mat &depth,
                      const float depth_scale,
                      const std::vector<float> &cam_in,
                      const std::vector<float> &cam_ex,
                      const cv::Mat &mask               )
{
    const float resolution_half = resolution * 0.5;

    PointCloudsfromDepth<PointT,TYPE_DEPTH>(
        seg2cloud, seg, depth, depth_scale, cam_in, cam_ex, 
        Mat::zeros(depth.rows, depth.cols, CV_8UC3), mask);

    ModelCoefficients coef_bg;    

    vector<int> labels;
    vector<typename PointCloud<PointT>::Ptr> clouds;
    for( typename map<uint8_t,typename PointCloud<PointT>::Ptr>::iterator
         it=seg2cloud.begin(); it!=seg2cloud.end(); it++                  )
    {
        if( it->first == 0) // background
        {
            if( it->second->size() > 3 )
            {
                PointIndices inliers;
                SACSegmentation<PointT> sac;
                sac.setOptimizeCoefficients (true);
                sac.setModelType (SACMODEL_PLANE);
                sac.setMethodType (SAC_RANSAC);
                sac.setDistanceThreshold (resolution);
                sac.setInputCloud (it->second);
                sac.segment(inliers, coef_bg);
            }            
        }
        else
        {
            labels.push_back(it->first);
            clouds.push_back(it->second);

            typename PointCloud<PointT>::Ptr cloud_hidden(new PointCloud<PointT>);
            seg2hidden.insert(pair<uint8_t,typename PointCloud<PointT>::Ptr>(
                              it->first, cloud_hidden)                        );
        }
    }

    PointClouds2Voxel<PointT>( clouds, voxel,
                               voxel_origin, n_x,n_y,n_z, resolution, labels );

    Eigen::Matrix4f tf_cam2world;
    if( cam_ex.size() == 0 )    
    {
        tf_cam2world = Eigen::Matrix4f::Identity();
    }
    else
    {        
        tf_cam2world << cam_ex[0],  cam_ex[1],  cam_ex[2],  cam_ex[3], 
                        cam_ex[4],  cam_ex[5],  cam_ex[6],  cam_ex[7], 
                        cam_ex[8],  cam_ex[9],  cam_ex[10], cam_ex[11], 
                        cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15];
    }

    Eigen::Matrix4f tf_world2cam;
    tf_world2cam = tf_cam2world.inverse();

    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {
        if( voxel[i_x][i_y][i_z] > 0 ) continue;

        PointT pt_world;
        pt_world.x = voxel_origin.x + i_x*resolution + resolution_half;
        pt_world.y = voxel_origin.y + i_y*resolution + resolution_half;
        pt_world.z = voxel_origin.z + i_z*resolution + resolution_half;

        if( coef_bg.values.size() ) // under ground
        {
            float dist = pt_world.x*coef_bg.values[0] + 
                         pt_world.y*coef_bg.values[1] + 
                         pt_world.z*coef_bg.values[2] + coef_bg.values[3];

            if( dist <= 0 )
            {
                voxel[i_x][i_y][i_z] = (int)INFINITY;
                continue;
            }             
        }

        PointT pt_cam;
        pt_cam.x = pt_world.x * tf_world2cam(0,0) + 
                   pt_world.y * tf_world2cam(0,1) + 
                   pt_world.z * tf_world2cam(0,2) + tf_world2cam(0,3);
        pt_cam.y = pt_world.x * tf_world2cam(1,0) + 
                   pt_world.y * tf_world2cam(1,1) + 
                   pt_world.z * tf_world2cam(1,2) + tf_world2cam(1,3);
        pt_cam.z = pt_world.x * tf_world2cam(2,0) + 
                   pt_world.y * tf_world2cam(2,1) + 
                   pt_world.z * tf_world2cam(2,2) + tf_world2cam(2,3);

        int c = (int)(pt_cam.x / pt_cam.z * cam_in[0] + cam_in[2]);
        int r = (int)(pt_cam.y / pt_cam.z * cam_in[4] + cam_in[5]);

        if( r < 0 || depth.rows <= r || 
            c < 0 || depth.cols <= c    ) continue;

        int label = seg.at<uint8_t>(r,c);

        float dep = (float)depth.at<TYPE_DEPTH>(r,c) * depth_scale;
        if( label != 0 && dep > 0 && dep < pt_cam.z )
        {
            voxel[i_x][i_y][i_z] = (int)-INFINITY; // hidden volume
            seg2hidden.find(label)->second->push_back(pt_world);
        }
        else
        {
            for( int rr=r-5; rr<r+5; rr++ )
            for( int cc=c-5; cc<c+5; cc++ )
            {                
                if( rr==r && cc==c ) continue;
                if( rr < 0 || depth.rows <= rr || 
                    cc < 0 || depth.cols <= cc    ) continue;

                int ll = seg.at<uint8_t>(rr,cc);
                float dd = (float)depth.at<TYPE_DEPTH>(rr,cc) * depth_scale;
                if( ll != 0 && dd > 0 && dd < pt_cam.z )
                {
                    voxel[i_x][i_y][i_z] = (int)-INFINITY; // hidden volume
                    seg2hidden.find(ll)->second->push_back(pt_world);
                    break;
                }
            }
        }
    }
#if 0
    visualization::PCLVisualizer viewer;
    int v1, v2, v3;
    viewer.createViewPort (0.00, 0.0, 0.33, 1.0, v1);
    viewer.createViewPort (0.33, 0.0, 0.66, 1.0, v2);
    viewer.createViewPort (0.66, 0.0, 0.99, 1.0, v3);
    viewer.setWindowName("debug");
    viewer.setSize(600,480);
    viewer.setPosition(600,0);
    viewer.setCameraPosition(-1,0,0,1,0,0,0,0,1);
    viewer.setBackgroundColor (0.2, 0.2, 0.2);
    viewer.addCoordinateSystem(0.1,"ref1", v1);
    viewer.addCoordinateSystem(0.1,"ref2", v2);
    viewer.addCoordinateSystem(0.1,"ref3", v3);    
    addVoxel(viewer, voxel, resolution, "voxel1", v1);
    addVoxel(viewer, voxel, resolution, "voxel2", v2, -INFINITY);
    addVoxel(viewer, voxel, resolution, "voxel3", v3,  INFINITY);    
    viewer.spin();
#endif    
}

template
void VoxelsfromDepth<PointXYZRGB, uint16_t>( 
                      voxel_t &voxel, 
                      PointXYZRGB &voxel_origin,
                      int &n_x, int &n_y, int &n_z,
                      const float resolution,                      
                      map<uint8_t,PointCloud<PointXYZRGB>::Ptr> &seg2cloud,
                      map<uint8_t,PointCloud<PointXYZRGB>::Ptr> &seg2hidden,                      
                      const cv::Mat &seg,
                      const cv::Mat &depth,
                      const float depth_scale,
                      const std::vector<float> &cam_in,
                      const std::vector<float> &cam_ex,
                      const cv::Mat &mask                );

void Voxel2PolygonMeshs( const voxel_t &voxel,                       
                         const float resolution,
                         map<uint8_t,PolygonMesh> &polymeshs )
{
    map<uint8_t,PointCloud<PointXYZRGB>::Ptr> clouds;    
    Voxel2PointClouds<PointXYZRGB>( voxel, clouds, resolution );

    polymeshs.clear();
    for( map<uint8_t,PointCloud<PointXYZRGB>::Ptr>::iterator 
         it = clouds.begin(); it != clouds.end(); it++ )
    {
        for( int p=0; p<it->second->size(); p++ )
        {
            uint8_t r = utils::colors_vis[it->first%20][0];
            uint8_t g = utils::colors_vis[it->first%20][1];
            uint8_t b = utils::colors_vis[it->first%20][2];

            it->second->points[p].r = r;
            it->second->points[p].g = g;
            it->second->points[p].b = b;            
        }

        PolygonMesh polymesh;
        ConvexHull<PointXYZRGB> chull;
        chull.setDimension(3);
        chull.setInputCloud (it->second);
        chull.reconstruct (polymesh);

        polymeshs.insert(pair<uint8_t,PolygonMesh>(it->first,polymesh));
    }
}

void GetTSDF( const vector<PolygonMesh> &polymeshs_in, 
              const vector<Eigen::Matrix4f> &poses,
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const vector<int> labels_in              )
{
    assert(polymeshs_in.size()==poses.size() && "polymeshs_in.size()==poses.size()");

    size_t n_objs = polymeshs_in.size();
    vector<PointCloud<PointXYZ>::Ptr> clouds_in(n_objs);
    for( int o=0; o<n_objs; o++ )
    {
        clouds_in[o] = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);        
        fromPCLPointCloud2(polymeshs_in[o].cloud, *clouds_in[o]);
        transformPointCloud(*clouds_in[o], *clouds_in[o], poses[o]);
    }
    
    GetTSDF<PointXYZ>( clouds_in,
                       voxel, tsdf, 
                       n_x, n_y, n_z,
                       offset_x, offset_y, offset_z, 
                       resolution, 
                       labels_in                     );
}

template <typename PointT>
void GetTSDF( const typename PointCloud<PointT>::Ptr &cloud_surf_in, 
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const vector<int> labels_in              )
{
    vector<typename PointCloud<PointT>::Ptr> clouds_surf(1);
    clouds_surf[0] = cloud_surf_in;
    
    GetTSDF<PointT>( clouds_surf,
                     voxel, tsdf, 
                     n_x, n_y, n_z,
                     offset_x, offset_y, offset_z, 
                     resolution, 
                     labels_in                     );
}

template <typename PointT>
void GetTSDF( const vector<typename PointCloud<PointT>::Ptr> &clouds_surf_in,
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const vector<int> labels_in              )
{

    typedef pair<PointT,PointT> PointMinMax;
    const float resolution_half = resolution * 0.5;
    const float tsdf_max =  resolution * 10;
    const float tsdf_min = -resolution * 10;

    size_t n_objs = clouds_surf_in.size();
    map<int,int> label2o;
    vector<int> labels;
    if( labels_in.size()==0 )
    {
        for( size_t o=0; o<n_objs; o++ )
        {
            labels.push_back(o+1);
        }
    }
    else
    {
        labels = labels_in;
    }
    for( int o=0; o<n_objs; o++ )
    {
        label2o.insert(pair<int,int>(labels[o],o));
    }

    PointCloud<PointT> cloud_minmax;
    vector<PolygonMesh> polymeshs(n_objs);
    vector<PointMinMax> minmax_objs(n_objs);
    vector<typename PointCloud<PointT>::Ptr> clouds_hull(n_objs);
    vector<typename PointCloud<PointT>::Ptr> clouds_surf(n_objs);
    
    ConvexHull<PointT> chull;
    for( size_t o=0; o<n_objs; o++ ) // ConvexHull cannot be parallized
    {
        // reduce the points and fill the holes
        PolygonMesh &polymesh = polymeshs[o];
        chull.setInputCloud(clouds_surf_in[o]);
        chull.reconstruct(polymesh);
    }

    #pragma omp parallel for
    for( size_t o=0; o<n_objs; o++ )
    {
        clouds_hull[o]
         = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);

        PolygonMesh &polymesh = polymeshs[o];
        fromPCLPointCloud2(polymesh.cloud, *clouds_hull[o]);
        getMinMax3D(*clouds_hull[o],minmax_objs[o].first,minmax_objs[o].second);
        
        cloud_minmax.push_back(minmax_objs[o].first);
        cloud_minmax.push_back(minmax_objs[o].second);

        // polymesh to a densy pointcloud on surface
        clouds_surf[o]
         = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);
        PolygonMesh2PointCloud(polymesh, *clouds_surf[o], resolution);
    }

    PointMinMax minmax_all;
    getMinMax3D(cloud_minmax, minmax_all.first, minmax_all.second);

    // decide the voxel size if needed
    if( n_x<=0         ||
        n_y<=0         ||
        n_z<=0         ||
        n_x==-INFINITY || n_x==INFINITY ||
        n_y==-INFINITY || n_y==INFINITY ||
        n_z==-INFINITY || n_z==INFINITY    )
    {
        float len_x = (minmax_all.second.x - minmax_all.first.x);
        float len_y = (minmax_all.second.y - minmax_all.first.y);
        float len_z = (minmax_all.second.z - minmax_all.first.z);

        n_x = ceil(len_x/resolution) * 2;
        n_y = ceil(len_y/resolution) * 2;
        n_z = ceil(len_z/resolution) * 2;
    }

    // decide the offset and voxel size if needed
    if( offset_x==-INFINITY || offset_x==INFINITY ||
        offset_y==-INFINITY || offset_y==INFINITY ||
        offset_z==-INFINITY || offset_z==INFINITY    )
    {
        PointT pt_center;
        pt_center.x = (minmax_all.first.x + minmax_all.second.x) * 0.5;
        pt_center.y = (minmax_all.first.y + minmax_all.second.y) * 0.5;
        pt_center.z = (minmax_all.first.z + minmax_all.second.z) * 0.5;

        offset_x = pt_center.x - n_x/2*resolution;
        offset_y = pt_center.y - n_y/2*resolution;
        //offset_z = pt_center.z - n_z/2*resolution;
        offset_z = minmax_all.first.z;
    }

    // resize and initialize voxel and tsdf
    voxel.resize(n_x);
     tsdf.resize(n_x);
    for( int i_x=0; i_x<n_x; i_x++ )
    {
        voxel[i_x].resize(n_y);
         tsdf[i_x].resize(n_y);
        for( int i_y=0; i_y<n_y; i_y++ )
        {
            voxel[i_x][i_y].resize(n_z);
             tsdf[i_x][i_y].resize(n_z);
            for( int i_z=0; i_z<n_z; i_z++ )
            {
                voxel[i_x][i_y][i_z] = 0;
                 tsdf[i_x][i_y][i_z] = tsdf_max;
            }
        }
    }    

    for( size_t o=0; o<n_objs; o++ )
    {
        // surface
        #pragma omp parallel for
        for( int p=0; p<clouds_surf[o]->size(); p++ )
        {
            PointT &pt = clouds_surf[o]->points[p];

            int i_x = (pt.x - offset_x) / resolution;
            int i_y = (pt.y - offset_y) / resolution;
            int i_z = (pt.z - offset_z) / resolution;

            voxel[i_x][i_y][i_z] = labels[o];
            tsdf[i_x][i_y][i_z]  = 0;
        }
    }
        
    vector<typename PointCloud<PointT>::Ptr> clouds_vox_surf(n_objs);    
    vector<typename PointCloud<PointT>::Ptr> clouds_box(n_objs);    
    vector<typename PointCloud<PointT>::Ptr> clouds_box_tight(n_objs);    
    for( size_t o=0; o<n_objs; o++ )
    {
        clouds_vox_surf[o]
         = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);
        clouds_box[o]
         = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);
        clouds_box_tight[o]
         = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);
    }
    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {
        PointT pt;
        pt.x = offset_x + i_x*resolution + resolution_half;
        pt.y = offset_y + i_y*resolution + resolution_half;
        pt.z = offset_z + i_z*resolution + resolution_half;

        if( tsdf[i_x][i_y][i_z] == 0 )
        {
            int o = label2o.find(voxel[i_x][i_y][i_z])->second;
            clouds_vox_surf[o]->push_back(pt);
        }
        else
        {
            for( size_t o=0; o<n_objs; o++ )
            {
                if( minmax_objs[o].first.x  - tsdf_max <= pt.x &&
                    minmax_objs[o].first.y  - tsdf_max <= pt.y &&
                    minmax_objs[o].first.z  - tsdf_max <= pt.z &&
                    minmax_objs[o].second.x + tsdf_max >= pt.x &&
                    minmax_objs[o].second.y + tsdf_max >= pt.y &&
                    minmax_objs[o].second.z + tsdf_max >= pt.z    )
                {
                    clouds_box[o]->push_back(pt);                
                }
                if( minmax_objs[o].first.x  <= pt.x &&
                    minmax_objs[o].first.y  <= pt.y &&
                    minmax_objs[o].first.z  <= pt.z &&
                    minmax_objs[o].second.x >= pt.x &&
                    minmax_objs[o].second.y >= pt.y &&
                    minmax_objs[o].second.z >= pt.z    )
                {
                    clouds_box_tight[o]->push_back(pt);                
                }
            }
        }
    }

    // fill inside
    #pragma omp parallel for
    for( size_t o=0; o<n_objs; o++ )
    {
        typename PointCloud<PointT>::Ptr cloud_fill(new PointCloud<PointT>);

        CropHull<PointT> cropHull;
        cropHull.setDim(3);    
        cropHull.setInputCloud(clouds_box_tight[o]);
        cropHull.setHullCloud(clouds_hull[o]);
        cropHull.setHullIndices(polymeshs[o].polygons);    
        cropHull.filter(*cloud_fill);

        RadiusOutlierRemoval<PointT> outrem;
        outrem.setInputCloud(cloud_fill);
        outrem.setRadiusSearch(resolution);
        outrem.setMinNeighborsInRadius (2);
        outrem.filter (*cloud_fill);

        // inside
        for( int p=0; p<cloud_fill->size(); p++ )
        {
            PointT &pt = cloud_fill->points[p];

            int i_x = (pt.x - offset_x) / resolution;
            int i_y = (pt.y - offset_y) / resolution;
            int i_z = (pt.z - offset_z) / resolution;

            voxel[i_x][i_y][i_z] = labels[o];
            tsdf[i_x][i_y][i_z]  = tsdf_min;
        }    
    }

    // cannot be parallized
    for( size_t o=0; o<n_objs; o++ )
    {
        KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(clouds_vox_surf[o]);

        for( int p=0; p<clouds_box[o]->size(); p++ )
        {
            PointT &pt = clouds_box[o]->points[p];

            int i_x = (pt.x - offset_x) / resolution;
            int i_y = (pt.y - offset_y) / resolution;
            int i_z = (pt.z - offset_z) / resolution;

            if( tsdf[i_x][i_y][i_z] == 0 ) continue;

            vector<int> idxes;
            vector<float> dists;
            kdtree.nearestKSearch(pt, 1, idxes, dists);
            
            if( tsdf[i_x][i_y][i_z] > 0 )
            {
                float val = sqrt(dists[0]);
                if( val > tsdf_max ) val = tsdf_max;
                if( tsdf[i_x][i_y][i_z] > val ) tsdf[i_x][i_y][i_z] = val;
            } 
            else
            {
                float val = -sqrt(dists[0]);
                if( val < tsdf_min ) val = tsdf_min;
                if( tsdf[i_x][i_y][i_z] < val ) tsdf[i_x][i_y][i_z] = val;
            }
        }
    }

#if 0
    visualization::PCLVisualizer viewer;
    int v1, v2, v3;
    viewer.createViewPort (0.00, 0.0, 0.33, 1.0, v1);
    viewer.createViewPort (0.33, 0.0, 0.66, 1.0, v2);
    viewer.createViewPort (0.66, 0.0, 0.99, 1.0, v3);
    viewer.setWindowName("debug");
    viewer.setSize(600,480);
    viewer.setPosition(600,0);
    viewer.setCameraPosition(-1,0,0,1,0,0,0,0,1);
    viewer.setBackgroundColor (0.2, 0.2, 0.2);
    viewer.addCoordinateSystem(0.1,"ref1", v1);
    viewer.addCoordinateSystem(0.1,"ref2", v2);
    viewer.addCoordinateSystem(0.1,"ref3", v3);
    for( size_t o=0; o<n_objs; o++ )
    {
        stringstream ss;
        ss << "cloud" << o;        
        viewer.addPointCloud(clouds_surf_in[o], ss.str(), v1);
        //viewer.addPolygonMesh(polymeshs[o], ss.str() + "polymesh", v1);
        //viewer.addPointCloud(clouds_fill[o], ss.str() + "cloud2", v2);
    }    
    addVoxel(viewer, voxel, resolution, "voxel", v2);
    addTSDF( viewer, tsdf,  resolution, "tsdf",  v3);
    viewer.spin();
#endif
}

template void GetTSDF<PointXYZ>( 
              const PointCloud<PointXYZ>::Ptr &cloud_surf_in, 
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const vector<int> labels_in                               );

template void GetTSDF<PointXYZRGB>( 
              const PointCloud<PointXYZRGB>::Ptr &cloud_surf_in, 
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const vector<int> labels_in                               );

template void GetTSDF<PointXYZ>(
              const vector<PointCloud<PointXYZ>::Ptr> &clouds_surf_in,
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const vector<int> labels_in                               );

template void GetTSDF<PointXYZRGB>(
              const vector<PointCloud<PointXYZRGB>::Ptr> &clouds_surf_in,
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const vector<int> labels_in                               );

void WriteVoxLabel_sscnet(const std::string &filename, 
                          vector<float> &vox_origin, vector<float> &cam_pose, 
                          voxel_t &voxel)
{
    assert(voxel.size()>0       && "voxel.size() > 0"       );
    assert(voxel[0].size()>0    && "voxel[0].size() > 0"    );
    assert(voxel[0][0].size()>0 && "voxel[0][0[].size() > 0");
    assert(vox_origin.size()==3 && "vox_origin.size()==3"    );
    assert(cam_pose.size()==16  && "cam_pose.size()==16");

    int n_x = voxel.size();
    int n_y = voxel[0].size();
    int n_z = voxel[0][0].size();

    // Open file
    std::ofstream ofs(filename, std::ios::binary);

    // Read voxel origin in world coordinates
    for (int i = 0; i < 3; ++i)
    {
        ofs.write((char*)&vox_origin[i], sizeof(float));
    }

    // Read camera pose
    for (int i = 0; i < 16; ++i)
    {
        ofs.write((char*)&cam_pose[i], sizeof(float));
    }

    // x y z -> z x y
    int totalsize = n_x*n_z*n_y;
    vector<int> voxel_sscnet(totalsize);

    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {        
        int val = voxel[i_x][i_y][i_z];
        //assert(val < 256 && "cannot exceed 255 (uint8)");
        
        /*
        int idx = i_x*n_z*n_y + 
                      i_z*n_y +
                          i_y ;
         */
        int idx = i_x*n_z*n_y + 
                      i_z*n_y +
                          i_y ;
        
        voxel_sscnet[idx] = val;
    }

    int idx=0;
    while( idx < totalsize )
    {
        int value = voxel_sscnet[idx];

        int count=1;
        while( idx+1 < totalsize &&
               //count < 256       &&
               value == voxel_sscnet[idx+1] )
        {
            idx++;
            count++;
        }

        ofs.write((char*)&value, sizeof(int));
        ofs.write((char*)&count, sizeof(int));

        idx++;
    }
}

void ReadVoxLabel_sscnet(const std::string &filename, 
                         vector<float> &vox_origin, vector<float> &cam_pose, 
                         voxel_t &voxel, 
                         const int n_x, const int n_y, const int n_z )
{
    voxel.resize(n_x);    
    for( int i_x=0; i_x<n_x; i_x++ )
    {
        voxel[i_x].resize(n_y);
        for( int i_y=0; i_y<n_y; i_y++ )
        {
            voxel[i_x][i_y].resize(n_z);
        }
    }
    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {
        voxel[i_x][i_y][i_z] = 0;
    }

    // Open file
    std::ifstream ifs(filename, std::ios::binary);

    // Read voxel origin in world coordinates
    vox_origin.resize(3);
    for (int i = 0; i < 3; ++i)
    {
        ifs.read((char*)&vox_origin[i], sizeof(float));
    }

    // Read camera pose
    cam_pose.resize(16);
    for (int i = 0; i < 16; ++i)
    {
        ifs.read((char*)&cam_pose[i], sizeof(float));
    }

    int sum=0;

    int totalsize = n_x*n_z*n_y;    
    int idx=0;
    vector<int> vox_RLE;
    while (!ifs.eof()) 
    {
        int value;
        ifs.read((char*)&value, sizeof(int));

        int count;
        ifs.read((char*)&count, sizeof(int));

        sum += count;

        while( idx < totalsize && count > 0 )
        {
            int i_x = idx / (n_z*n_y);
            int i_y = (idx / n_z) % n_y;
            int i_z = idx % n_z;

            if( value==255 ) voxel[i_x][i_y][i_z] = 0;
            else             voxel[i_x][i_y][i_z] = value;

            idx++;
            count--;            
        }
    }
}

static void Erosion( const Mat &src, Mat &dst, int erosion_size=10 );
static void Erosion( const Mat &src, Mat &dst, int erosion_size)
{  
    int erosion_type = MORPH_RECT;
    Mat element = getStructuringElement( erosion_type,
                    Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                    Point( erosion_size, erosion_size ) );

    /// Apply the erosion operation
    erode( src, dst, element );    
}

static void Dilation( const Mat &src, Mat &dst, int dilation_size=10 );
static void Dilation( const Mat &src, Mat &dst, int dilation_size )
{  
    int dilation_type = MORPH_RECT;
    Mat element = getStructuringElement( dilation_type,
                    Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                    Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate( src, dst, element );  
}

void FillHole( const Mat &src, Mat &dst, const int size )
{
    Mat img_e, img_d;
    Erosion(src, img_e, size);
    Dilation(img_e, img_d, size);
    dst = img_d;
}

template<typename Scalar>
std::set<Scalar> Unique(const cv::Mat& input)
{
    std::set<Scalar> out;
    for( int r=0; r<input.rows; r++)
    for( int c=0; c<input.cols; c++)
    {
        Scalar value = input.at<Scalar>(r,c);
        out.insert(value);
    }

    return out;
}

template std::set<uint8_t>  Unique<uint8_t>(const cv::Mat& input);
template std::set<uint16_t> Unique<uint16_t>(const cv::Mat& input);

float float16tofloat32(char* data)
{    
    uint16_t num16 = *(uint16_t*)data;

    if( num16 == 0x7E00 || num16 == 0x7F00 ) return NAN;
    if( num16 == 0x7C00 ) return  INFINITY;
    if( num16 == 0x7C00 ) return -INFINITY;
    
    uint32_t num32 = 0, sgn, exp, man;
    sgn = ((((0x8000 & num16)    )           ) << 16);
    exp = ((((0x7C00 & num16)>>10) + (127-15)) << 23);
    man = ((((0x03FF & num16)    )           ) << 13);
    num32 = sgn | exp | man;

    return *(float*)(&num32);
}

template<typename PointT>
void FindCorners2D( const PointCloud<PointT> &cloud_prj,
                    vector<PointT> &pts_corner)
{

    float dist_max = -1;
    pts_corner.resize(4);
    vector<int> p_corners(4);
    for( int p=0; p<cloud_prj.size(); p++ )
    {
        const PointT &pt = cloud_prj[p];
        float dist = compute_dist(cloud_prj[0],pt);
        if( dist_max < dist )
        {
            p_corners[0] = p;
            dist_max = dist;
        }
    }
    pts_corner[0] = cloud_prj[p_corners[0]];

    dist_max = -1;
    for( int p=0; p<cloud_prj.size(); p++ )
    {
        const PointT &pt = cloud_prj[p];
        float dist = compute_dist((const PointT)pts_corner[0],pt);
        if( dist_max < dist )
        {
            p_corners[1] = p;
            dist_max = dist;
        }
    }
    pts_corner[1] = cloud_prj[p_corners[1]];

    dist_max = -1;
    for( int p=0; p<cloud_prj.size(); p++ )
    {
        const PointT &pt = cloud_prj[p];
        float dist = compute_dist_Point2LineSegment(pt, pts_corner[0], pts_corner[1]);
        if( dist_max < dist )
        {
            p_corners[2] = p;
            dist_max = dist;
        }
    }
    pts_corner[2] = cloud_prj[p_corners[2]];

    dist_max = -1;
    for( int p=0; p<cloud_prj.size(); p++ )
    {
        const PointT &pt = cloud_prj[p];
        float dist = compute_dist((const PointT)pts_corner[2],pt);
        if( dist_max < dist )
        {
            p_corners[3] = p;
            dist_max = dist;
        }
    }
    pts_corner[3] = cloud_prj[p_corners[3]];
}

template
void FindCorners2D<PointXYZ>( const PointCloud<PointXYZ> &cloud_prj,
                              vector<PointXYZ> &pts_corner           );

template
void FindCorners2D<PointXYZRGB>( const PointCloud<PointXYZRGB> &cloud_prj,
                                 vector<PointXYZRGB> &pts_corner );

template<typename PointT>
void FindBoundary2D( const typename PointCloud<PointT>::Ptr cloud_prj,
                     vector<uint32_t> &idxes_boundary,
                     PointCloud<PointT> &cloud_boundary,
                     const float resolution)
{
    typename PointCloud<PointT>::Ptr cloud_hull(new PointCloud<PointT>);
    PolygonMesh polymesh;
    ConvexHull<PointT> chull;
    chull.setDimension(2);
    chull.setInputCloud(cloud_prj);
    chull.reconstruct (polymesh);    
    fromPCLPointCloud2(polymesh.cloud, *cloud_hull);
    vector<uint32_t> &vert = polymesh.polygons[0].vertices;

    typename PointCloud<PointT>::Ptr cloud_line(new PointCloud<PointT>);
    float val_sum = 0;
    float dist_sum = 0;
    for( int v=0; v<vert.size(); v++ )
    {
        int v_nxt = (v + 1 + vert.size()) % vert.size();

        PointT pt_cur = cloud_hull->points[vert[v]];
        PointT pt_nxt = cloud_hull->points[vert[v_nxt]];

        Eigen::Vector3f vec( pt_nxt.x-pt_cur.x,
                             pt_nxt.y-pt_cur.y,
                             pt_nxt.z-pt_cur.z );
        vec.normalize();

        float dist = compute_dist(pt_cur,pt_nxt);
        int n_pts = (int)(dist / resolution);
        cloud_line->push_back(pt_cur);
        for( int p=0; p<n_pts; p++ )
        {
            PointT pt;
            pt.x = pt_cur.x + vec[0]*resolution*(p+1);
            pt.y = pt_cur.y + vec[1]*resolution*(p+1);
            pt.z = pt_cur.z + vec[2]*resolution*(p+1);

            cloud_line->push_back(pt);
        }
    }

    vector<int> idxes;
    vector<float> dists;
    KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_prj);
    idxes_boundary.clear();
    cloud_boundary.clear();    
    for( int p=0; p<cloud_line->size(); p++ )
    {
        kdtree.nearestKSearch(cloud_line->points[p], 1, idxes, dists);
        idxes_boundary.push_back(idxes[0]);
        cloud_boundary.push_back(cloud_prj->points[idxes[0]]);        
    }
}

template<typename PointT>
void FindBoundary2D( const typename PointCloud<PointT>::Ptr cloud_prj,
                     PointCloud<PointT> &cloud_boundary,                     
                     const float resolution)
{
    vector<uint32_t> idxes_boundary;
    FindBoundary2D( cloud_prj, idxes_boundary, cloud_boundary, resolution);
}

template<typename PointT>
void FindBoundary2D( const typename PointCloud<PointT>::Ptr cloud_prj,
                     vector<uint32_t> &idxes_boundary,
                     const float resolution)
{
    PointCloud<PointT> cloud_boundary;
    FindBoundary2D( cloud_prj, idxes_boundary, cloud_boundary, resolution);
}

template
void FindBoundary2D<PointXYZ>( 
    const PointCloud<PointXYZ>::Ptr cloud_prj,
    vector<uint32_t> &idxes_boundary,
    PointCloud<PointXYZ> &cloud_boundary,
    const float resolution                      );

template
void FindBoundary2D<PointXYZ>( 
    const PointCloud<PointXYZ>::Ptr cloud_prj,
    vector<uint32_t> &idxes_boundary,    
    const float resolution                      );

template
void FindBoundary2D<PointXYZ>( 
    const PointCloud<PointXYZ>::Ptr cloud_prj,
    PointCloud<PointXYZ> &cloud_boundary,
    const float resolution                      );

template
void FindBoundary2D<PointXYZRGB>( 
    const PointCloud<PointXYZRGB>::Ptr cloud_prj,
    vector<uint32_t> &idxes_boundary,
    PointCloud<PointXYZRGB> &cloud_boundary,
    const float resolution                      );

template
void FindBoundary2D<PointXYZRGB>( 
    const PointCloud<PointXYZRGB>::Ptr cloud_prj,
    vector<uint32_t> &idxes_boundary,
    const float resolution                      );

template
void FindBoundary2D<PointXYZRGB>( 
    const PointCloud<PointXYZRGB>::Ptr cloud_prj,
    PointCloud<PointXYZRGB> &cloud_boundary,
    const float resolution                      );

void AverageQuaternion(vector<Eigen::Quaternionf> &qs, Eigen::Quaternionf &mean)
{
    Eigen::MatrixXf Q(4,qs.size());
    for( size_t q=0; q<qs.size(); q++ )
    {
        Q(0,q)=qs[q].w(); Q(1,q)=qs[q].x(); Q(2,q)=qs[q].y(); Q(3,q)=qs[q].z();
    }

    Eigen::MatrixXf QQt = Q * Q.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(4);
    es.compute(QQt);
    Eigen::MatrixXf axes = es.eigenvectors();
    Eigen::Vector4f vec = axes.col(3); // the vector with the largest eigenvalue
    mean = Eigen::Quaternionf(vec(0), vec(1), vec(2), vec(3));
    mean.normalize();
}

}