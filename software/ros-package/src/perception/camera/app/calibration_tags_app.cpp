#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>

#include <MoveItNode.hpp>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "camera/camera_config.hpp"
#include "camera/camera_utils.hpp"

using namespace std;
using namespace pcl;
using namespace changkyu;

static MoveItNode* moveit_node;

typedef struct TAG
{
    int id;
    string name;
    geometry_msgs::Pose pose;
} TAG;

void ParseParam( const ros::NodeHandle &nh, 
                 std::vector<TAG> &tags,
                 std::vector<geometry_msgs::Pose> &poses_cam )
{
    XmlRpc::XmlRpcValue tag_list;
    nh.getParam("/changkyu/camera/calibration/tags", tag_list);
    ROS_ASSERT(tag_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for( XmlRpc::XmlRpcValue::ValueStruct::const_iterator 
         it = tag_list.begin(); it != tag_list.end(); ++it) 
    {
        string name = it->first;
        string key_id   = "/changkyu/camera/calibration/tags/"+ name +"/id";
        string key_ori  = "/changkyu/camera/calibration/tags/"+ name +"/orientation";
        string key_pos  = "/changkyu/camera/calibration/tags/"+ name +"/position";
        
        TAG tag;
        tag.name = name;      
        nh.getParam(key_id        , tag.id);
        nh.getParam(key_pos + "/x", tag.pose.position.x);
        nh.getParam(key_pos + "/y", tag.pose.position.y);
        nh.getParam(key_pos + "/z", tag.pose.position.z);
        nh.getParam(key_ori + "/x", tag.pose.orientation.x);
        nh.getParam(key_ori + "/y", tag.pose.orientation.y);
        nh.getParam(key_ori + "/z", tag.pose.orientation.z);
        nh.getParam(key_ori + "/w", tag.pose.orientation.w);        
        tags.push_back(tag);
    }

    XmlRpc::XmlRpcValue pose_list;
    nh.getParam("/changkyu/camera/calibration/cam_poses", pose_list);
    ROS_ASSERT(pose_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for( XmlRpc::XmlRpcValue::ValueStruct::const_iterator 
         it = pose_list.begin(); it != pose_list.end(); ++it) 
    {
        string name = it->first;
        string key_ori  = "/changkyu/camera/calibration/cam_poses/"+ name +"/orientation";
        string key_pos  = "/changkyu/camera/calibration/cam_poses/"+ name +"/position";
        
        geometry_msgs::Pose pose;
        nh.getParam(key_pos + "/x", pose.position.x);
        nh.getParam(key_pos + "/y", pose.position.y);
        nh.getParam(key_pos + "/z", pose.position.z);
        nh.getParam(key_ori + "/x", pose.orientation.x);
        nh.getParam(key_ori + "/y", pose.orientation.y);
        nh.getParam(key_ori + "/z", pose.orientation.z);
        nh.getParam(key_ori + "/w", pose.orientation.w);        
        poses_cam.push_back(pose);
    }
}

int main(int argc, char* argv[])
{
    // ROS init
    ros::init(argc,argv,"changkyu_workspace_tags_app");    
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    vector<TAG> tags_wld;
    vector<geometry_msgs::Pose> poses_cam;
    ParseParam(nh,tags_wld,poses_cam);

    cout << "=== Tag Info ===" << endl;
    for( int i=0; i<tags_wld.size(); i++ )
    {
        cout << "name: " << tags_wld[i].name << endl;
        cout << tags_wld[i].pose << endl;
    }
    cout << "================" << endl;



    cout << "[ROBOT] Goto Home" << endl;
    moveit_node = new MoveItNode;
    moveit_node->goto_home();
    vector<TAG> tags_cam, tags_trs;
    for( int i=0; i<poses_cam.size(); i++ )
    {
        cout << "[ROBOT] Goto " << endl;
        cout << poses_cam[i] << endl;
        moveit_node->plan_and_execute(poses_cam[i]);
        ros::Duration(2).sleep();

        tf::Transform trs(tf::Quaternion(poses_cam[i].orientation.x,
                                         poses_cam[i].orientation.y,
                                         poses_cam[i].orientation.z,
                                         poses_cam[i].orientation.w),
                             tf::Vector3(poses_cam[i].position.x,
                                         poses_cam[i].position.y,
                                         poses_cam[i].position.z)    );
        
        cout << "obtain tag position ..." << endl;

        ar_track_alvar_msgs::AlvarMarkers::ConstPtr arPoseMarkers_ptr;
        try
        {
            arPoseMarkers_ptr = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", nh);
        }
        catch(std::exception &e)
        {
            ROS_ERROR("Exception during waitForMessage from AlvarMakers: %s", e.what());
        }

        for( int m=0; m<arPoseMarkers_ptr->markers.size(); m++ )
        {
            TAG tag;
            tag.id = arPoseMarkers_ptr->markers[m].id;
            tag.pose.position    = arPoseMarkers_ptr->markers[m].pose.pose.position;
            tag.pose.orientation = arPoseMarkers_ptr->markers[m].pose.pose.orientation;
            tags_cam.push_back(tag);

            tf::Vector3 xyz_trs = trs(tf::Vector3(tag.pose.position.x,
                                                  tag.pose.position.y,
                                                  tag.pose.position.z));

            TAG tag_trs;
            tag_trs.id = tag.id;
            tag_trs.pose.position.x = xyz_trs.getX();
            tag_trs.pose.position.y = xyz_trs.getY();
            tag_trs.pose.position.z = xyz_trs.getZ();
            tags_trs.push_back(tag_trs);

            cout << "id: "   << arPoseMarkers_ptr->markers[m].id << endl;
            cout << "pose: " << arPoseMarkers_ptr->markers[m].pose.pose.position << endl;
        }
    }

    int n_points = tags_cam.size();
    cv::Mat points_cam(n_points,3,CV_32FC1);
    cv::Mat points_trs(n_points,3,CV_32FC1);
    cv::Mat points_wld(n_points,3,CV_32FC1);
    for( int i=0; i<n_points; i++)
    {
        points_cam.at<float>(i,0) = tags_cam[i].pose.position.x;
        points_cam.at<float>(i,1) = tags_cam[i].pose.position.y;
        points_cam.at<float>(i,2) = tags_cam[i].pose.position.z;
        points_trs.at<float>(i,0) = tags_trs[i].pose.position.x;
        points_trs.at<float>(i,1) = tags_trs[i].pose.position.y;
        points_trs.at<float>(i,2) = tags_trs[i].pose.position.z;
        points_wld.at<float>(i,0) = tags_wld[tags_cam[i].id].pose.position.x;
        points_wld.at<float>(i,1) = tags_wld[tags_cam[i].id].pose.position.y;
        points_wld.at<float>(i,2) = tags_wld[tags_cam[i].id].pose.position.z;
    }


    cout << points_trs << endl;
    cout << points_cam << endl;
    cout << points_wld << endl;

    cv::Mat mx_trs2wld, inliers;    
    cv::estimateAffine3D(points_trs,points_wld,mx_trs2wld,inliers);
    cout << mx_trs2wld << endl;

    /*
    bool running = true;
    ros::Rate r(30);
    while( ros::ok() && running )
    {
        ros::spinOnce();
    }
    */
    moveit_node->goto_home();
    ros::shutdown();
}
