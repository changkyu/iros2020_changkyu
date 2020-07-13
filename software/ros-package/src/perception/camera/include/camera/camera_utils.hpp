#ifndef CAMERA_UTILS__HPP__
#define CAMERA_UTILS__HPP__

#include <thread>

#include "camera/camera_config.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/utils.hpp"

namespace changkyu
{

bool CropWorkspace( const camerainfo_t &caminfo, 
                    const pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                    pcl::PointCloud<pcl::PointXYZRGB> &cloud_crop,
                    cv::Mat& image_crop,
                    cv::Mat& mask_crop  );

bool CropWorkspace( const camerainfo_t &caminfo, 
                    const pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                    pcl::PointCloud<pcl::PointXYZRGB> &cloud_crop      );

bool GetInputFromCamera( ros::NodeHandle &nh,
                         sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,
                         cv::Mat &image,
                         cv::Mat &depth,
                         pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace=false     );

bool GetInputFromCamera( ros::NodeHandle &nh,
                         sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,                         
                         pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace=false      );

bool GetInputFromCamera( ros::NodeHandle &nh,
                         cv::Mat &image,
                         cv::Mat &depth,
                         pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace=false      );

bool GetInputFromCamera( ros::NodeHandle &nh,
                         cv::Mat &image,
                         cv::Mat &depth,
                         pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace,
                         cv::Mat &image_crop,
                         cv::Mat &mask_crop                         );

Eigen::Matrix4f GetTransformCameraOnHand(ros::NodeHandle &nh, camerainfo_t &caminfo);

bool GetInputFromCameraOnHand(ros::NodeHandle &nh,
                              cv::Mat &image, cv::Mat &depth,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_world,
                              camerainfo_t &caminfo, 
                              Eigen::Matrix4f &tf_cam2world,
                              const bool crop_workspace, 
                              cv::Mat &image_crop, 
                              cv::Mat &mask_crop,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop  );

bool GetInputFromCameras( ros::NodeHandle &nh,
                          pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                          std::map<std::string,camerainfo_t> &name2camerainfo,
                          const bool crop_workspace=false              );

bool GetInputFromImage( const cv::Mat &image,
                        const cv::Mat &depth,
                        const camerainfo_t &caminfo,
                        sensor_msgs::Image &msg_image,
                        sensor_msgs::Image &msg_depth,
                        pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                        const bool crop = false );

bool GetInputFromImage( const cv::Mat &image,
                        const cv::Mat &depth,
                        const camerainfo_t &caminfo,
                        pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                        const bool crop = false );

bool GetInputFromImage( const sensor_msgs::Image &msg_image,
                        const sensor_msgs::Image &msg_depth,
                        const camerainfo_t &caminfo,
                        pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                        const bool crop = false );

typedef class CameraManager
{
public:
    CameraManager( ros::NodeHandle &nh_in,
                   const float resolution_voxel );
    ~CameraManager();

    void SetCropWorkspace(const bool crop);
    void SetCropWorkspace(std::string name, const bool crop);
    bool GetInputFromCamera( const std::string &camera_name, 
                             cv::Mat &image, cv::Mat &depth );
    bool GetInputFromCamera( const std::string &camera_name,
                             sensor_msgs::Image &msg_image,
                             sensor_msgs::Image &msg_depth,                                        
                             cv::Mat &image,
                             cv::Mat &depth,
                             pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam );
    bool GetNewInputFromCamera( const std::string camera_name, 
                                sensor_msgs::Image &msg_image,
                                sensor_msgs::Image &msg_depth,                                        
                                cv::Mat &image,
                                cv::Mat &depth,
                                pcl::PointCloud<pcl::PointXYZRGB> &cloud_new );
    
    void StartRecording( const std::string &camera_name, 
                          const std::string &prefix, double sec );
    void StopRecording();
    void ThreadRecording( const std::string &camera_name,
                          const std::string &prefix, double sec);


private:
    ros::NodeHandle nh;
    pcl::PointXYZRGB voxel_origin;    
    int n_x, n_y, n_z;
    utils::voxel_t voxel_prv;
    std::map<std::string,camerainfo_t> name2camerainfo;
    std::map<std::string,bool> crop_workspace;
    float resolution;

    bool running_recording;
    std::thread* t_recording;

} CameraManager;


}

#endif