#include <thread>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <MoveItNode.hpp>
//include <Grasper.hpp>

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
PointCloud<PointXYZRGB>::Ptr cloud_cam(new PointCloud<PointXYZRGB>);

#define ROBOT (1)
#if ROBOT

double ref_points[18][3] = {
 {0.396625,-0.209541,0.36},
 {0.457483,-0.244678,0.36},
 {0.518342,-0.279815,0.36},
 {0.347942,-0.293863,0.36},
 {0.408800,-0.329000,0.36},
 {0.469658,-0.364137,0.36},
 {0.299258,-0.378185,0.36},
 {0.360117,-0.413322,0.36},
 {0.420975,-0.448459,0.36},
 {0.396625,-0.209541,0.46},
 {0.457483,-0.244678,0.46},
 {0.518342,-0.279815,0.46},
 {0.347942,-0.293863,0.46},
 {0.408800,-0.329000,0.46},
 {0.469658,-0.364137,0.46},
 {0.299258,-0.378185,0.46},
 {0.360117,-0.413322,0.46},
 {0.420975,-0.448459,0.46}
};
int idx_ref=0;

static MoveItNode* moveit_node;
//static GraspNode* grasp_node;
static iiwa_ros::iiwaRos* my_iiwa;
static geometry_msgs::Pose pose_default;
#endif

void UpdateViewer()
{
    vector<cv::Mat> images(name2camerainfo.size());
    vector<cv::Mat> depths(name2camerainfo.size());

    int i=0;    
    for( map<string,camerainfo_t>::iterator it = name2camerainfo.begin();
         it != name2camerainfo.end(); it++                                )
    {
        vector<float> &workspace = it->second.workspace;
        vector<float> &camera_RT = it->second.camera_RT;
        Eigen::Matrix4f tf;
        tf << camera_RT[0],  camera_RT[1],  camera_RT[2],  camera_RT[3],
              camera_RT[4],  camera_RT[5],  camera_RT[6],  camera_RT[7],
              camera_RT[8],  camera_RT[9],  camera_RT[10], camera_RT[11],
              camera_RT[12], camera_RT[13], camera_RT[14], camera_RT[15];
                
        changkyu::GetInputFromCamera( *nh, images[i], depths[i], *cloud_cam, it->second );
                
        transformPointCloud(*cloud_cam,*cloud_cam,tf);
        
        stringstream ss;
        ss << "camera" << i;
        viewer->updatePointCloud(cloud_cam, ss.str() + "cloud");        
        i++;
    }
}

void Callback_pointpicking (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    if (event.getPointIndex () == -1)
    {
      return;
    }

    PointXYZRGB point = cloud_cam->points[event.getPointIndex()];
    
    viewer->removeAllShapes();
    PointXYZRGB pt_x1 ,pt_x2, pt_y1, pt_y2, pt_z1, pt_z2;
    pt_x1 = point; pt_x2 = point;
    pt_y1 = point; pt_y2 = point;
    pt_z1 = point; pt_z2 = point;
    pt_x1.x = point.x - 0.01; pt_x2.x = point.x + 0.01;
    pt_y1.y = point.y - 0.01; pt_y2.y = point.y + 0.01;
    pt_z1.z = point.z - 0.01; pt_z2.z = point.z + 0.01;

    viewer->addLine(pt_x1, pt_x2, 0,0,1, "pt_x" );
    viewer->addLine(pt_y1, pt_y2, 0,0,1, "pt_y" );
    viewer->addLine(pt_z1, pt_z2, 0,0,1, "pt_z" );

    geometry_msgs::PoseStamped::ConstPtr pose_ptr
     = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "/iiwa/state/CartesianPose", *nh);    

    ROS_INFO_STREAM("select point: " << point << endl << "robot pose: " << pose_ptr->pose);
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
            UpdateViewer();
            cout << "[done]" << endl;
        }
        else if( event.getKeySym()=="Return" )
        {
            geometry_msgs::Pose pose = pose_default;
            pose.position.x = ref_points[idx_ref][0];
            pose.position.y = ref_points[idx_ref][1];
            pose.position.z = ref_points[idx_ref][2];

            moveit_node->plan_and_execute(pose);

            idx_ref = (idx_ref+1) % 18;
            UpdateViewer();
        }        
    }
}

int main(int argc, char* argv[])
{
    // ROS init
    ros::init(argc,argv,"changkyu_calibration_tune_app");    
    nh = new ros::NodeHandle;
    nh->getParam("/changkyu/camera_config", fp_config);

    ParseParam(fp_config, name2camerainfo);

    ros::AsyncSpinner spinner(1);
    spinner.start();

#if ROBOT
    idx_ref = 0;
    
    pose_default.position.x = 0.39;
    pose_default.position.y = -0.34;
    pose_default.position.z = 0.55;
    pose_default.orientation.x = -0.38268343;
    pose_default.orientation.y = 0.92387953;
    pose_default.orientation.z = 0.0;
    pose_default.orientation.w = 0.0;    

    my_iiwa = new iiwa_ros::iiwaRos;
    my_iiwa->init();
    moveit_node = new MoveItNode;
    //grasp_node = new GraspNode;

    //grasp_node->publish_command("o");
    moveit_node->goto_home();
#endif

    int n_camera = name2camerainfo.size();
    vs.resize(n_camera);    
    viewer = new visualization::PCLVisualizer;;
    viewer->setWindowName("debug");
    viewer->setSize(640*n_camera,960);
    viewer->setPosition(0,0);
    viewer->setCameraPosition(2,0,1,0.5,0,-0.2,0,0,1);    
    float col_unit = 1.0/(float)n_camera;
    for( int i=0; i<n_camera; i++ )
    {
        stringstream ss;
        ss << "camera" << i;

        viewer->createViewPort(col_unit*i,0.0,col_unit*(i+1),1.0,vs[i]);        
        viewer->setBackgroundColor (0.2,0.2,0.2, vs[i]);
        viewer->addCoordinateSystem(0.1, "ref_" + ss.str(), vs[i]);    
        viewer->addPointCloud(cloud_cam, ss.str() + "cloud", vs[i]);        
    }
    viewer->registerKeyboardCallback(Callback_pclkeyboard, viewer);
    viewer->registerPointPickingCallback (Callback_pointpicking, viewer); 
    
    UpdateViewer();
    viewer->spin();

    delete viewer;
    delete nh;
}
