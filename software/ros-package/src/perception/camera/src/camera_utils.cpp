#include "camera/camera_utils.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

#include <tf/transform_datatypes.h>

using namespace std;
using namespace pcl;

namespace changkyu
{

bool CropWorkspace( const std::vector<float> &workspace,
                    const std::vector<float> &camera_RT,
                    const PointCloud<PointXYZRGB> &cloud_cam,
                    PointCloud<PointXYZRGB> &cloud_crop,
                    const cv::Mat &image,
                    cv::Mat &image_crop,
                    cv::Mat &mask_crop,
                    bool isOrganized=false  )
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

    Eigen::Matrix4f tf;
    tf << camera_RT[0],  camera_RT[1],  camera_RT[2],  camera_RT[3], 
          camera_RT[4],  camera_RT[5],  camera_RT[6],  camera_RT[7], 
          camera_RT[8],  camera_RT[9],  camera_RT[10], camera_RT[11], 
          camera_RT[12], camera_RT[13], camera_RT[14], camera_RT[15];

    PointCloud<PointXYZRGB>::Ptr cloud_tmp(new PointCloud<PointXYZRGB>);
    transformPointCloud(cloud_cam, *cloud_tmp, tf_ws.inverse()*tf);

    double x_min = -workspace[7]*0.5;
    double x_max =  workspace[7]*0.5;
    double y_min = -workspace[8]*0.5;
    double y_max =  workspace[8]*0.5;
    double z_min = -workspace[9]*0.5;
    double z_max =  workspace[9]*0.5;

    vector<int> idxes_crop;
    PointCloud<PointXYZRGB> cloud_out;
    for( size_t p=0; p<cloud_tmp->size(); p++ )
    {
        if( 0 < cloud_cam.points[p].z && // valid depth
            x_min <= cloud_tmp->points[p].x &&
            x_max >= cloud_tmp->points[p].x &&
            y_min <= cloud_tmp->points[p].y &&
            y_max >= cloud_tmp->points[p].y &&
            z_min <= cloud_tmp->points[p].z &&
            z_max >= cloud_tmp->points[p].z    )
        {
            cloud_out.push_back(cloud_cam[p]);
            idxes_crop.push_back(p);
        }
        else if( isOrganized )
        {
            PointXYZRGB pt;
            pt.x=0; pt.y=0; pt.z=0;
            cloud_out.push_back(pt);
        }  
    }
    // cloud_cam & cloud_crop could refer the same memory space
    copyPointCloud(cloud_out, cloud_crop);

    if( image.cols > 0 && image.rows > 0 )
    {
        mask_crop = cv::Mat::zeros(image.rows,image.cols,CV_8UC1);
        image_crop = cv::Mat::zeros(image.rows,image.cols,CV_8UC3);
        for( size_t i=0; i<idxes_crop.size(); i++ )
        {
            int p = idxes_crop[i];
            int r = p / image.cols;
            int c = p % image.cols;
            mask_crop.at<uint8_t>(r,c) = 255;
            image_crop.at<cv::Vec3b>(r,c) = image.at<uint8_t>(r,c);
        }
    }
}

bool CropWorkspace( const camerainfo_t &caminfo, 
                    const PointCloud<PointXYZRGB> &cloud_cam,
                    PointCloud<PointXYZRGB> &cloud_crop,
                    const cv::Mat &image,
                    cv::Mat &image_crop,
                    cv::Mat &mask_crop  )
{
    CropWorkspace( caminfo.workspace, caminfo.camera_RT, 
                   cloud_cam, cloud_crop, image, image_crop, mask_crop);
}

bool CropWorkspace( const camerainfo_t &caminfo, 
                    const PointCloud<PointXYZRGB> &cloud_cam,
                    PointCloud<PointXYZRGB> &cloud_crop )
{
    cv::Mat image, image_crop, mask_crop;
    CropWorkspace(caminfo, cloud_cam, cloud_crop, image, image_crop, mask_crop);
}

bool GetInputFromCamera( ros::NodeHandle &nh,
                         cv::Mat &image,
                         cv::Mat &depth,
                         camerainfo_t &caminfo )
{
    sensor_msgs::Image::ConstPtr img_ptr, dep_ptr;
    sensor_msgs::CameraInfo::ConstPtr ci_depth;
    try
    {        
        img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_image, nh, ros::Duration(1));
        if( img_ptr==NULL ) return false;
        dep_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_depth, nh);
        ci_depth = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo.topic_camerainfo,nh);
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Exception during waitForMessage from camera: %s", e.what());
        return false;
    }

    caminfo.camera_K.resize(9);
    for( int k=0; k<9; k++ ) caminfo.camera_K[k] = ci_depth->K[k];
    
    cv_bridge::CvImagePtr cv_img
     = cv_bridge::toCvCopy(*img_ptr, sensor_msgs::image_encodings::RGB8);
    image = cv_img->image.clone();

    cv_bridge::CvImagePtr cv_dep
     = cv_bridge::toCvCopy(*dep_ptr, dep_ptr->encoding);
    depth = cv_dep->image.clone();
}

bool GetInputFromCamera( ros::NodeHandle &nh,
                         sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,
                         cv::Mat &image,
                         cv::Mat &depth,
                         PointCloud<PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace,
                         cv::Mat &image_crop,
                         cv::Mat &mask_crop         )
{
    sensor_msgs::Image::ConstPtr img_ptr, dep_ptr;
    sensor_msgs::CameraInfo::ConstPtr ci_depth;
    try
    {
        img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_image, nh, ros::Duration(10));
        if( img_ptr==NULL ) return false;
        dep_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_depth, nh);
        ci_depth = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo.topic_camerainfo,nh);
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Exception during waitForMessage from camera: %s", e.what());
        return false;
    }

    msg_image = *img_ptr;
    msg_depth = *dep_ptr;

    caminfo.camera_K.resize(9);
    for( int k=0; k<9; k++ ) caminfo.camera_K[k] = ci_depth->K[k];
    
    cv_bridge::CvImagePtr cv_img
     = cv_bridge::toCvCopy(*img_ptr, sensor_msgs::image_encodings::RGB8);
    image = cv_img->image.clone();

    cv_bridge::CvImagePtr cv_dep
     = cv_bridge::toCvCopy(*dep_ptr, dep_ptr->encoding);
    depth = cv_dep->image.clone();

    int bitdepth = sensor_msgs::image_encodings::bitDepth(dep_ptr->encoding);
    if( bitdepth==16 )
    {
        utils::PointCloudfromDepth<PointXYZRGB, uint16_t>(
            cloud_cam, cv_dep->image, caminfo.depth_scale,
            caminfo.camera_K, vector<float>(), cv_img->image, cv::Mat(), true );
    }
    else
    {        
        return false;
    }

    if( crop_workspace && caminfo.workspace.size()==10 )
    {
        CropWorkspace( caminfo, cloud_cam, cloud_cam, image, image_crop, mask_crop );
        cloud_cam.width = cloud_cam.size();
        cloud_cam.height = 1;
    }

    return true;
}

bool GetInputFromCamera( ros::NodeHandle &nh,
                         sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,
                         cv::Mat &image,
                         cv::Mat &depth,
                         PointCloud<PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace )
{
    cv::Mat image_crop, mask_crop;
    return GetInputFromCamera( nh, 
                               msg_image, msg_depth, image, depth, cloud_cam, 
                               caminfo, crop_workspace, image_crop, mask_crop);
}

bool GetInputFromCamera( ros::NodeHandle &nh,
                         sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,                         
                         PointCloud<PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace           )
{
    cv::Mat image, depth, image_crop, mask_crop;
    return GetInputFromCamera( nh, 
                               msg_image, msg_depth, image, depth, cloud_cam, 
                               caminfo, crop_workspace, image_crop, mask_crop);
}

bool GetInputFromCamera( ros::NodeHandle &nh,
                         cv::Mat &image,
                         cv::Mat &depth,
                         PointCloud<PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace           )
{
    sensor_msgs::Image msg_image;
    sensor_msgs::Image msg_depth;
    cv::Mat image_crop, mask_crop;
    return GetInputFromCamera( nh, 
                               msg_image, msg_depth, image, depth, cloud_cam, 
                               caminfo, crop_workspace, image_crop, mask_crop);
}

bool GetInputFromCamera( ros::NodeHandle &nh,
                         cv::Mat &image,
                         cv::Mat &depth,                         
                         PointCloud<PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo,
                         const bool crop_workspace,
                         cv::Mat &image_crop,
                         cv::Mat &mask_crop                  )
{
    sensor_msgs::Image msg_image;
    sensor_msgs::Image msg_depth;
    return GetInputFromCamera( nh, 
                               msg_image, msg_depth, image, depth, cloud_cam, 
                               caminfo, crop_workspace, image_crop, mask_crop );
}

bool GetInputFromCameras( ros::NodeHandle &nh,
                          pcl::PointCloud<pcl::PointXYZRGB> &cloud_cam,
                          std::map<std::string,camerainfo_t> &name2camerainfo,
                          const bool crop_workspace           )
{
    cv::Mat image;
    cv::Mat depth;
    sensor_msgs::Image msg_image;
    sensor_msgs::Image msg_depth;
    
    bool res = true;
    for( map<string,camerainfo_t>::iterator it = name2camerainfo.begin();
         it != name2camerainfo.end(); it++                                )
    {    
        PointCloud<PointXYZRGB> cloud;
        res &= GetInputFromCamera( nh, 
                                   msg_image, msg_depth, image, depth, cloud, 
                                   it->second, crop_workspace);

        vector<float> &camera_RT = it->second.camera_RT;
        Eigen::Matrix4f tf;
        tf << camera_RT[0],  camera_RT[1],  camera_RT[2],  camera_RT[3],
              camera_RT[4],  camera_RT[5],  camera_RT[6],  camera_RT[7],
              camera_RT[8],  camera_RT[9],  camera_RT[10], camera_RT[11],
              camera_RT[12], camera_RT[13], camera_RT[14], camera_RT[15];
        
        transformPointCloud(cloud,cloud,tf);
        cloud_cam += cloud;
    }

    return res;
}

bool GetInputFromImage( const cv::Mat &image,
                        const cv::Mat &depth,
                        const camerainfo_t &caminfo,
                        sensor_msgs::Image &msg_image,
                        sensor_msgs::Image &msg_depth,
                        PointCloud<PointXYZRGB> &cloud_cam,
                        const bool crop )
{
    cv_bridge::CvImage cvimg_image;    
    cvimg_image.encoding = sensor_msgs::image_encodings::BGR8;
    cvimg_image.image    = image;
    msg_image = *cvimg_image.toImageMsg();

    cv_bridge::CvImage cvimg_depth;
    cvimg_depth.encoding = sensor_msgs::image_encodings::MONO16;
    cvimg_depth.image    = depth;
    msg_depth = *cvimg_depth.toImageMsg();

    utils::PointCloudfromDepth<PointXYZRGB, uint16_t>(
            cloud_cam, depth, caminfo.depth_scale,
            caminfo.camera_K, vector<float>(), image, cv::Mat(), true );

    if( crop && caminfo.workspace.size()==10 )
    {
        CropWorkspace( caminfo, cloud_cam, cloud_cam );
        cloud_cam.width = cloud_cam.size();
        cloud_cam.height = 1;
    }

    return true;
}

bool GetInputFromImage( const cv::Mat &image,
                        const cv::Mat &depth,
                        const camerainfo_t &caminfo,
                        PointCloud<PointXYZRGB> &cloud_cam,
                        const bool crop )
{
    utils::PointCloudfromDepth<PointXYZRGB, uint16_t>(
            cloud_cam, depth, caminfo.depth_scale,
            caminfo.camera_K, vector<float>(), image, cv::Mat(), true );

    if( crop && caminfo.workspace.size()==10 )
    {
        CropWorkspace( caminfo, cloud_cam, cloud_cam );
        cloud_cam.width = cloud_cam.size();
        cloud_cam.height = 1;
    }

    return true;
}

bool GetInputFromImage( const sensor_msgs::Image &msg_image,
                        const sensor_msgs::Image &msg_depth,
                        const camerainfo_t &caminfo,
                        PointCloud<PointXYZRGB> &cloud_cam,
                        const bool crop )
{
    cv_bridge::CvImagePtr cv_img
     = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cv_img->image.clone();

    cv_bridge::CvImagePtr cv_dep
     = cv_bridge::toCvCopy(msg_depth, msg_depth.encoding);
    cv::Mat depth = cv_dep->image.clone();

    utils::PointCloudfromDepth<PointXYZRGB, uint16_t>(
            cloud_cam, depth, caminfo.depth_scale,
            caminfo.camera_K, vector<float>(), image, cv::Mat(), true );

    if( crop && caminfo.workspace.size()==10 )
    {
        CropWorkspace( caminfo, cloud_cam, cloud_cam );
        cloud_cam.width = cloud_cam.size();
        cloud_cam.height = 1;
    }

    return true;
}

Eigen::Matrix4f GetTransformCameraOnHand(ros::NodeHandle &nh, camerainfo_t &caminfo)
{
    // Get transformation
    Eigen::Matrix4f tf_yx;
    tf_yx << 0,1,0,0,
            -1,0,0,0,
             0,0,1,0,
             0,0,0,1;

    geometry_msgs::PoseStamped::ConstPtr pose_ee_cur
     = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/iiwa/state/CartesianPose",nh,ros::Duration(10));
    Eigen::Matrix3f tf_rot
     = Eigen::Quaternionf(pose_ee_cur->pose.orientation.w,
                          pose_ee_cur->pose.orientation.x,
                          pose_ee_cur->pose.orientation.y,
                          pose_ee_cur->pose.orientation.z).toRotationMatrix();
    Eigen::Matrix4f tf_ee;
    tf_ee << tf_rot(0,0),tf_rot(0,1),tf_rot(0,2),pose_ee_cur->pose.position.x,
             tf_rot(1,0),tf_rot(1,1),tf_rot(1,2),pose_ee_cur->pose.position.y,
             tf_rot(2,0),tf_rot(2,1),tf_rot(2,2),pose_ee_cur->pose.position.z,
                       0,          0,          0,                           1;
    return tf_ee*caminfo.tf_cam2world*tf_yx;
}

bool GetInputFromCameraOnHand(ros::NodeHandle &nh, 
                              cv::Mat &image, 
                              cv::Mat &depth,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_world,
                              camerainfo_t &caminfo, 
                              Eigen::Matrix4f &tf_cam2world,
                              const bool crop_workspace, 
                              cv::Mat &image_crop, 
                              cv::Mat &mask_crop, 
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop )
{
    // Get image, depth, pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZRGB>);    
    bool ret = GetInputFromCamera(nh, image, depth, *cloud_cam, caminfo, false); 

    if( !ret ) return false;

    Eigen::Matrix4f tf = GetTransformCameraOnHand(nh,caminfo);
    tf_cam2world = tf;
    if( crop_workspace )
    {
        vector<float> RT = {tf(0,0),tf(0,1),tf(0,2),tf(0,3),
                            tf(1,0),tf(1,1),tf(1,2),tf(1,3),
                            tf(2,0),tf(2,1),tf(2,2),tf(2,3),
                            tf(3,0),tf(3,1),tf(3,2),tf(3,3) };

        // organized cloud
        CropWorkspace( caminfo.workspace, RT, *cloud_cam, *cloud_cam, image, image_crop, mask_crop, true );
    }

    transformPointCloud(*cloud_cam,*cloud_world,tf);
    copyPointCloud(*cloud_world,*cloud_crop);

    for( int p=0; p<cloud_cam->size(); p++ )
    {
        if(cloud_cam->points[p].z==0)
        {
             cloud_crop->points[p].x = 0;
             cloud_crop->points[p].y = 0;
             cloud_crop->points[p].z = 0;
        }
    }

    cloud_world->width  = depth.cols;
    cloud_world->height = depth.rows;
    cloud_crop->width  = depth.cols;
    cloud_crop->height = depth.rows;

    return true;
}

CameraManager::CameraManager( ros::NodeHandle &nh_in,                              
                              const float resolution_voxel )
{
    nh = nh_in;        
    voxel_origin.x = INFINITY;
    voxel_origin.y = INFINITY;
    voxel_origin.z = INFINITY;
    n_x=0;
    n_y=0;
    n_z=0;
    ParseParam(nh, name2camerainfo);    
    resolution = resolution_voxel;

    for( map<string,camerainfo_t>::iterator 
         it = name2camerainfo.begin(); it != name2camerainfo.end(); it++ )
    {
        if( it->second.workspace.size() ) SetCropWorkspace(it->first ,true);        
    }

    t_recording = NULL;
}

CameraManager::~CameraManager(){}

void CameraManager::ThreadRecording( const string &camera_name, 
                                     const string &prefix_save, double sec )
{
    camerainfo_t &caminfo = name2camerainfo.find(camera_name)->second;

    int idx_save = 0;
    while(running_recording)
    {
        clock_t begin = clock();

        char name[256];
        sprintf(name,"%s.%06d",prefix_save.c_str(),idx_save); 
        
        cv::Mat image, depth;
        changkyu::GetInputFromCamera(nh, image, depth, caminfo);

        // Image
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);        
        imwrite(string(name) + ".color.png", image);        
        imwrite(string(name) + ".depth.png", depth);        
        ROS_INFO_STREAM("Saved " << name);
        
        idx_save++;

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

        std::this_thread::sleep_for(
            std::chrono::milliseconds((long int)(sec-elapsed_secs)/1000));            
    }
}

void CameraManager::StartRecording( const string &camera_name, 
                                    const string &prefix_save, double sec )
{
    StopRecording();
    running_recording = true;
    t_recording = new thread( &CameraManager::ThreadRecording, this, 
                              camera_name, prefix_save, sec          );
}

void CameraManager::StopRecording()
{
    running_recording = false;
    if( t_recording )
    {        
        t_recording->join();
        delete t_recording;
        t_recording = NULL;
    }
}

void CameraManager::SetCropWorkspace(string name, const bool crop)
{
    std::map<std::string,bool>::iterator it = crop_workspace.find(name);
    if( it == crop_workspace.end() )
        crop_workspace.insert(std::pair<std::string,bool>(name, crop));
    else
        it->second = crop;

    camerainfo_t &caminfo = name2camerainfo.find(name)->second;
    vector<float> &workspace = caminfo.workspace;
    vector<float> &camera_RT = caminfo.camera_RT;

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
    PointCloud<PointXYZRGB> cloud_ws;
    cloud_ws.points.resize(8);
    cloud_ws[0].x =  workspace[7]*0.5;
    cloud_ws[1].x =  workspace[7]*0.5;
    cloud_ws[2].x = -workspace[7]*0.5;
    cloud_ws[3].x = -workspace[7]*0.5;
    cloud_ws[4].x =  workspace[7]*0.5;
    cloud_ws[5].x =  workspace[7]*0.5;
    cloud_ws[6].x = -workspace[7]*0.5;
    cloud_ws[7].x = -workspace[7]*0.5;

    cloud_ws[0].y =  workspace[8]*0.5;
    cloud_ws[1].y = -workspace[8]*0.5;
    cloud_ws[2].y =  workspace[8]*0.5;
    cloud_ws[3].y = -workspace[8]*0.5;
    cloud_ws[4].y =  workspace[8]*0.5;
    cloud_ws[5].y = -workspace[8]*0.5;
    cloud_ws[6].y =  workspace[8]*0.5;
    cloud_ws[7].y = -workspace[8]*0.5;

    cloud_ws[0].z =  workspace[9]*0.5;
    cloud_ws[1].z =  workspace[9]*0.5;
    cloud_ws[2].z =  workspace[9]*0.5;
    cloud_ws[3].z =  workspace[9]*0.5;
    cloud_ws[4].z = -workspace[9]*0.5;
    cloud_ws[5].z = -workspace[9]*0.5;
    cloud_ws[6].z = -workspace[9]*0.5;
    cloud_ws[7].z = -workspace[9]*0.5;

    transformPointCloud(cloud_ws, cloud_ws, tf_ws);

    PointXYZRGB pt_min, pt_max;    
    getMinMax3D(cloud_ws, pt_min, pt_max);

    voxel_origin.x = pt_min.x - resolution;
    voxel_origin.y = pt_min.y - resolution;
    voxel_origin.z = pt_min.z - resolution;
    int n_x = (pt_max.x - pt_min.x) / resolution + 2;
    int n_y = (pt_max.y - pt_min.y) / resolution + 2;
    int n_z = (pt_max.z - pt_min.z) / resolution + 2;
}

void CameraManager::SetCropWorkspace(const bool crop)
{
    for( map<string,camerainfo_t>::iterator 
         it = name2camerainfo.begin(); it != name2camerainfo.end(); it++ )
    {
        SetCropWorkspace(it->first ,crop);
    }
}

bool CameraManager::GetInputFromCamera( const string &camera_name, 
                                        cv::Mat &image, cv::Mat &depth )
{
    camerainfo_t &caminfo = name2camerainfo.find(camera_name)->second;
    changkyu::GetInputFromCamera(nh, image, depth, caminfo);
}

bool CameraManager::GetInputFromCamera( const string &camera_name,
                                        sensor_msgs::Image &msg_image,
                                        sensor_msgs::Image &msg_depth,                                        
                                        cv::Mat &image,
                                        cv::Mat &depth,
                                        PointCloud<PointXYZRGB> &cloud_cam )
{
    map<string,camerainfo_t>::iterator it_caminfo = name2camerainfo.find(camera_name);
    if( it_caminfo == name2camerainfo.end() )
    {
        cerr << "[Error] cannot find " << camera_name << endl;
        return false;
    }

    bool crop;
    map<string,bool>::iterator it_ws = crop_workspace.find(camera_name);
    if( it_ws == crop_workspace.end() ) crop = false;
    else                                crop = it_ws->second;

    return changkyu::GetInputFromCamera( nh, msg_image, msg_depth, image, depth, 
                                         cloud_cam, it_caminfo->second, crop  );
}

bool CameraManager::GetNewInputFromCamera( const string camera_name, 
                                           sensor_msgs::Image &msg_image,
                                           sensor_msgs::Image &msg_depth,
                                           cv::Mat &image,
                                           cv::Mat &depth,
                                           PointCloud<PointXYZRGB> &cloud_new )
{    
    PointCloud<PointXYZRGB> cloud_cam;
    bool ret = GetInputFromCamera( camera_name, 
                                   msg_image, msg_depth, image, depth, 
                                   cloud_cam                           );

    cloud_new.clear();
    if( voxel_prv.size() )
    {
        for( int p=0; p<cloud_cam.size(); p++ )
        {
            PointXYZRGB &pt = cloud_cam[p];

            int i_x = (pt.x - voxel_origin.x) / resolution;
            int i_y = (pt.y - voxel_origin.y) / resolution;
            int i_z = (pt.z - voxel_origin.z) / resolution;

            int cnt = 0;
            for( int i_x_nei=i_x-1; i_x_nei<=i_x+1; i_x_nei++ )
            for( int i_y_nei=i_y-1; i_y_nei<=i_y+1; i_y_nei++ )
            for( int i_z_nei=i_z-1; i_z_nei<=i_z+1; i_z_nei++ )                
            {
                if( i_x_nei < 0 || n_x <= i_x_nei || 
                    i_y_nei < 0 || n_y <= i_y_nei || 
                    i_z_nei < 0 || n_z <= i_z_nei    ) continue;
                
                if( voxel_prv[i_x_nei][i_y_nei][i_z_nei]==1 ) cnt++;                 
            }

            if( cnt < 9 ) cloud_new.push_back(pt);
        }
    }
    else
    {
        copyPointCloud(cloud_cam, cloud_new);
    }
    
    utils::PointCloud2Voxel( cloud_cam, voxel_prv, voxel_origin,
                             n_x, n_y, n_z, resolution, 1        );

    return ret;
}

}
