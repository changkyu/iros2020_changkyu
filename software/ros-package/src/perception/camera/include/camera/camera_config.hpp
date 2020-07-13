#ifndef CAMERA_CONFIG__HPP
#define CAMERA_CONFIG__HPP

#include <string>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Geometry>

namespace changkyu
{

typedef struct camerainfo_t
{
    std::string name;
    std::string topic_image;
    std::string topic_depth;
    std::string topic_camerainfo;
    float depth_scale;
    std::vector<float> camera_K;
    std::vector<float> camera_RT;
    Eigen::Matrix4f tf_cam2world;
    std::vector<float> workspace;
    std::string param_segmentation;    
} camerainfo_t;

bool UpdateIntrinsicParam(ros::NodeHandle &nh, camerainfo_t &caminfo);

void ParseParam(ros::NodeHandle &nh, 
                std::map<std::string,camerainfo_t> &name2camerainfo);

void ParseParam( const std::string &fp_yaml, 
                 std::map<std::string,camerainfo_t> &name2camerainfo );

}
#endif
