#include <thread>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include "camera/camera_config.hpp"
#include "camera/camera_utils.hpp"
#include "utils/utils.hpp"
#include "utils/utils_visualization.hpp"

using namespace std;
using namespace pcl;
using namespace changkyu;

static ros::NodeHandle* nh;
static vector<int> vs;
static visualization::PCLVisualizer* viewer;
static bool running;
static string fp_config;
static map<string,camerainfo_t> name2camerainfo;

#define CAMERA (1)

void UpdateViewer(bool save=false)
{
    vector<cv::Mat> images(name2camerainfo.size());
    vector<cv::Mat> depths(name2camerainfo.size());

    for( int i=0; i<2; i++ )
    {
        viewer->removeAllShapes(vs[i]);
    }

    PointCloud<PointXYZRGB>::Ptr cloud_cam_all(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloud_cam_crop_all(new PointCloud<PointXYZRGB>);

    PointXYZRGB pt11, pt12;
    pt11.x = 0.235; pt11.y = 0.-0.445; pt11.z = -0.25;
    pt12.x = 0.235; pt12.y = 0.-0.445; pt12.z =  0.00;
    PointXYZRGB pt21, pt22;
    pt21.x = 0.410; pt21.y = 0.-0.160; pt21.z = -0.25;
    pt22.x = 0.410; pt22.y = 0.-0.160; pt22.z =  0.00;
    PointXYZRGB pt31, pt32;
    pt31.x = 0.470; pt31.y = 0.-0.575; pt31.z = -0.25;
    pt32.x = 0.470; pt32.y = 0.-0.575; pt32.z =  0.00;
    viewer->addLine(pt11,pt12,1,0,0,"pt1",vs[0]);
    viewer->addLine(pt21,pt22,0,1,0,"pt2",vs[0]);
    viewer->addLine(pt31,pt32,0,0,1,"pt3",vs[0]);

    int i=0;    
    for( map<string,camerainfo_t>::iterator it = name2camerainfo.begin();
         it != name2camerainfo.end(); it++                                )
    {
        stringstream ss;
        ss << "camera_cloud_" << i;

        vector<float> &workspace = it->second.workspace;
        vector<float> &camera_RT = it->second.camera_RT;
        Eigen::Matrix4f tf;
        tf << camera_RT[0],  camera_RT[1],  camera_RT[2],  camera_RT[3],
              camera_RT[4],  camera_RT[5],  camera_RT[6],  camera_RT[7],
              camera_RT[8],  camera_RT[9],  camera_RT[10], camera_RT[11],
              camera_RT[12], camera_RT[13], camera_RT[14], camera_RT[15];
                
        cout << tf << endl;
        utils::addWorkspace(*viewer,workspace,1,1,1,ss.str() + "workspace",vs[0]);

        PointCloud<PointXYZRGB>::Ptr cloud_cam(new PointCloud<PointXYZRGB>);
#if CAMERA        
        changkyu::GetInputFromCamera( *nh, images[i], depths[i], *cloud_cam, it->second );
#else
        cout << __LINE__ << endl;
        images[i] = cv::imread("/home/cs1080/camera2_000000_color.png", cv::IMREAD_COLOR);
        cout << __LINE__ << endl;
        depths[i] = cv::imread("/home/cs1080/camera2_000000_depth.png", cv::IMREAD_ANYDEPTH);
        cout << __LINE__ << endl;
        cv::imshow("image", images[i]);
        cv::imshow("depth", depths[i]);
        cv::waitKey();
        changkyu::GetInputFromImage( images[i], depths[i], it->second, *cloud_cam, false );
        cout << __LINE__ << endl;
#endif
        
        PointCloud<PointXYZRGB>::Ptr cloud_cam_crop(new PointCloud<PointXYZRGB>);
        changkyu::CropWorkspace(it->second, *cloud_cam, *cloud_cam_crop);
        
        transformPointCloud(*cloud_cam,*cloud_cam,tf);
        transformPointCloud(*cloud_cam_crop,*cloud_cam_crop,tf);

        *cloud_cam_all      += *cloud_cam;
        *cloud_cam_crop_all += *cloud_cam_crop;

        i++;
    }

    for( i=0; i<2; i++ )
    {
        stringstream ss;
        ss << "camera_cloud_" << i;
        
        if( i==0 )
            viewer->updatePointCloud(cloud_cam_all,      ss.str());
        else
            viewer->updatePointCloud(cloud_cam_crop_all, ss.str());
    }

    if( save )
    {
        static int idx = 0;
        i=0;
        for( map<string,camerainfo_t>::iterator 
             it = name2camerainfo.begin();
             it != name2camerainfo.end();  it++ )
        {        
            char tmp[256];
            sprintf(tmp,"%s/%s_%06d",getenv("HOME"),it->first.c_str(),idx);

            string fp_save_image = string(tmp) + "_color.png";
            string fp_save_depth = string(tmp) + "_depth.png";

            cout << fp_save_image << endl;
            
            cv::cvtColor(images[i], images[i], CV_RGB2BGR);
            imwrite(fp_save_image, images[i]);
            cout << fp_save_depth << endl;

            imwrite(fp_save_depth, depths[i]);
            i++;
        }

        idx++;
    }
}

void Callback_pclkeyboard (const visualization::KeyboardEvent &event,
                           void* viewer_void)
{
    visualization::PCLVisualizer *viewer
     = static_cast<visualization::PCLVisualizer *> (viewer_void);
    if (event.keyDown())
    {
        if( event.getKeySym()=="Escape" )
        {
            running = false;            
        }
        else if( event.getKeySym()=="F5" )
        {
            cout << "[refresh] config ... " << endl;
            ParseParam(fp_config, name2camerainfo);
            UpdateViewer(false);
            cout << "[done]" << endl;
        }
        else if( event.getKeySym()=="Return" )
        {
            cout << "[save] images ... " << endl;            
            UpdateViewer(true);
            cout << "[done]" << endl;       
        }        
    }
}

void Callback_pointPicking(const pcl::visualization::PointPickingEvent &event)
{
    float x,y,z;
    event.getPoint(x,y,z);
    cout << "pick: " << x << ", " << y << ", " << z << endl;
}

int main(int argc, char* argv[])
{
    // ROS init
    ros::init(argc,argv,"changkyu_workspace_tune_app");    
    nh = new ros::NodeHandle;
    nh->getParam("/changkyu/camera_config", fp_config);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ParseParam(fp_config, name2camerainfo);

    vs.resize(2);
    viewer = new visualization::PCLVisualizer;;
    viewer->setWindowName("debug");
    viewer->setSize(640*2,480*2);
    viewer->setPosition(0,0);
    viewer->setCameraPosition(2,0,1,0.5,0,-0.2,0,0,1);
    for( int i=0; i<2; i++ )
    {
        stringstream ss;
        ss << "camera_cloud_" << i;

        viewer->createViewPort(0.0,0.5*i,1.0,0.5*(i+1),vs[i]);
        viewer->setBackgroundColor (0.2,0.2,0.2);
        viewer->addCoordinateSystem(1.0);

        PointCloud<PointXYZRGB>::Ptr cloud_cam(new PointCloud<PointXYZRGB>);
        viewer->addPointCloud(cloud_cam, ss.str(), vs[i]);
    }
    viewer->registerKeyboardCallback(Callback_pclkeyboard, viewer);
    viewer->registerPointPickingCallback(Callback_pointPicking);

    UpdateViewer(false);
    running = true;
    ros::Rate r(30);
    while( ros::ok() && running )
    {
        ros::spinOnce();
        viewer->spinOnce(1);
    }
    ros::shutdown();

    delete viewer;
    delete nh;
}
