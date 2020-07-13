#include "camera/camera_config.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;

namespace changkyu
{

bool UpdateIntrinsicParam(ros::NodeHandle &nh, camerainfo_t &caminfo)
{    
    sensor_msgs::CameraInfo::ConstPtr ci
     = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo.topic_camerainfo,nh,ros::Duration(1));
    
    if( ci==NULL ) return false;

    caminfo.camera_K.resize(9);
    for( int k=0; k<9; k++ ) caminfo.camera_K[k] = ci->K[k];

    return true;
}

void ParseParam(ros::NodeHandle &nh, 
                map<string,camerainfo_t> &name2camerainfo)
{
    name2camerainfo.clear();

    XmlRpc::XmlRpcValue camera_list;
    nh.getParam("/changkyu/camera", camera_list);
    ROS_ASSERT(camera_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for( XmlRpc::XmlRpcValue::ValueStruct::const_iterator 
         it = camera_list.begin(); it != camera_list.end(); ++it) 
    {    
        string name = it->first;
        string key_topic_image = "/changkyu/camera/"+ name +"/topic_image";
        string key_topic_depth = "/changkyu/camera/"+ name +"/topic_depth";
        string key_topic_camerainfo = "/changkyu/camera/"+ name +"/topic_image_camera_info";
        string key_depth_scale = "/changkyu/camera/"+ name +"/depth_scale";
        string key_camera_RT   = "/changkyu/camera/"+ name +"/camera_extrinsic";
        string key_workspace   = "/changkyu/camera/"+ name +"/workspace";
        string key_param_seg   = "/changkyu/camera/"+ name +"/param_segmentation";

        camerainfo_t caminfo;
        caminfo.name = name;
        nh.getParam(key_topic_image, caminfo.topic_image);
        nh.getParam(key_topic_depth, caminfo.topic_depth);
        nh.getParam(key_topic_camerainfo, caminfo.topic_camerainfo);
        nh.getParam(key_depth_scale, caminfo.depth_scale);
        nh.getParam(key_camera_RT,   caminfo.camera_RT);
        nh.getParam(key_workspace,   caminfo.workspace);
        nh.getParam(key_param_seg,   caminfo.param_segmentation);

        UpdateIntrinsicParam(nh, caminfo);
        if( caminfo.camera_RT.size()==7 )
        {
            vector<float> pose = caminfo.camera_RT;
            Eigen::Matrix3f tf_rot
             = Eigen::Quaternionf(pose[3],
                                  pose[4],
                                  pose[5],
                                  pose[6]).toRotationMatrix();
            
            caminfo.tf_cam2world << tf_rot(0,0),tf_rot(0,1),tf_rot(0,2),pose[0],
                                    tf_rot(1,0),tf_rot(1,1),tf_rot(1,2),pose[1],
                                    tf_rot(2,0),tf_rot(2,1),tf_rot(2,2),pose[2],
                                              0,          0,          0,      1;

            caminfo.camera_RT.resize(16);
            caminfo.camera_RT[0]  = tf_rot(0,0);
            caminfo.camera_RT[1]  = tf_rot(0,1);
            caminfo.camera_RT[2]  = tf_rot(0,2);
            caminfo.camera_RT[3]  = pose[0];
            caminfo.camera_RT[4]  = tf_rot(1,0);
            caminfo.camera_RT[5]  = tf_rot(1,1);
            caminfo.camera_RT[6]  = tf_rot(1,2);
            caminfo.camera_RT[7]  = pose[1];
            caminfo.camera_RT[8]  = tf_rot(2,0);
            caminfo.camera_RT[9]  = tf_rot(2,1);
            caminfo.camera_RT[10] = tf_rot(2,2);
            caminfo.camera_RT[11] = pose[2];
            caminfo.camera_RT[12] = 0;
            caminfo.camera_RT[13] = 0;
            caminfo.camera_RT[14] = 0;
            caminfo.camera_RT[15] = 1;
        }
        else if( caminfo.camera_RT.size()==16 )
        {
            caminfo.tf_cam2world << caminfo.camera_RT[0], 
                                    caminfo.camera_RT[1],
                                    caminfo.camera_RT[2],
                                    caminfo.camera_RT[3],
                                    caminfo.camera_RT[4],
                                    caminfo.camera_RT[5],
                                    caminfo.camera_RT[6],
                                    caminfo.camera_RT[7],
                                    caminfo.camera_RT[8],
                                    caminfo.camera_RT[9],
                                    caminfo.camera_RT[10],
                                    caminfo.camera_RT[11],
                                    caminfo.camera_RT[12],
                                    caminfo.camera_RT[13],
                                    caminfo.camera_RT[14],
                                    caminfo.camera_RT[15];
        }

        name2camerainfo.insert(
            pair<string,camerainfo_t>(name,caminfo) );
    }    
}

void ParseParam( const string &fp_yaml, 
                 map<string,camerainfo_t> &name2camerainfo )
{
    name2camerainfo.clear();

    YAML::Node params = YAML::LoadFile(fp_yaml);
    for( YAML::Node::iterator it = params.begin(); it != params.end(); it++ )
    {
        string name = it->first.as<string>();        
        YAML::Node &param = it->second;

        camerainfo_t caminfo;
        caminfo.name = name;
        caminfo.topic_image = param["topic_image"].as<string>();
        caminfo.topic_depth = param["topic_depth"].as<string>();
        caminfo.topic_camerainfo = param["topic_image_camera_info"].as<string>();
        caminfo.depth_scale = param["depth_scale"].as<float>();
        caminfo.camera_RT = param["camera_extrinsic"].as<vector<float> >();
        caminfo.workspace = param["workspace"].as<vector<float> >();
        caminfo.param_segmentation = param["param_segmentation"].as<string>();

        if( param["camera_intrinsic"] )
        {
            caminfo.camera_K = param["camera_intrinsic"].as<vector<float> >();
        }

        if( caminfo.camera_RT.size()==7 )
        {
            vector<float> pose = caminfo.camera_RT;
            Eigen::Matrix3f tf_rot
             = Eigen::Quaternionf(pose[3],
                                  pose[4],
                                  pose[5],
                                  pose[6]).toRotationMatrix();

            caminfo.tf_cam2world << tf_rot(0,0),tf_rot(0,1),tf_rot(0,2),pose[0],
                                    tf_rot(1,0),tf_rot(1,1),tf_rot(1,2),pose[1],
                                    tf_rot(2,0),tf_rot(2,1),tf_rot(2,2),pose[2],
                                              0,          0,          0,      1;

            caminfo.camera_RT.resize(16);
            caminfo.camera_RT[0]  = tf_rot(0,0);
            caminfo.camera_RT[1]  = tf_rot(0,1);
            caminfo.camera_RT[2]  = tf_rot(0,2);
            caminfo.camera_RT[3]  = pose[0];
            caminfo.camera_RT[4]  = tf_rot(1,0);
            caminfo.camera_RT[5]  = tf_rot(1,1);
            caminfo.camera_RT[6]  = tf_rot(1,2);
            caminfo.camera_RT[7]  = pose[1];
            caminfo.camera_RT[8]  = tf_rot(2,0);
            caminfo.camera_RT[9]  = tf_rot(2,1);
            caminfo.camera_RT[10] = tf_rot(2,2);
            caminfo.camera_RT[11] = pose[2];
            caminfo.camera_RT[12] = 0;
            caminfo.camera_RT[13] = 0;
            caminfo.camera_RT[14] = 0;
            caminfo.camera_RT[15] = 1;
        }
        else if( caminfo.camera_RT.size()==16 )
        {
            caminfo.tf_cam2world << caminfo.camera_RT[0], 
                                    caminfo.camera_RT[1],
                                    caminfo.camera_RT[2],
                                    caminfo.camera_RT[3],
                                    caminfo.camera_RT[4],
                                    caminfo.camera_RT[5],
                                    caminfo.camera_RT[6],
                                    caminfo.camera_RT[7],
                                    caminfo.camera_RT[8],
                                    caminfo.camera_RT[9],
                                    caminfo.camera_RT[10],
                                    caminfo.camera_RT[11],
                                    caminfo.camera_RT[12],
                                    caminfo.camera_RT[13],
                                    caminfo.camera_RT[14],
                                    caminfo.camera_RT[15];
        }

        name2camerainfo.insert(
            pair<string,camerainfo_t>(name,caminfo) );
    }
}

}
