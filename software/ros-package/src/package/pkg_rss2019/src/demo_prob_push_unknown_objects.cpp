#include <math.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include <Eigen/Geometry>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <MoveItNode.hpp>

#include <ar_track_alvar/MarkerDetector.h>
#include <aruco.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "camera/camera_config.hpp"
#include "camera/camera_utils.hpp"

#include "pkg_rss2019/add_object_srv.h"
#include "pkg_rss2019/estimate_prop_srv.h"
#include "pkg_rss2019/plan_prob_pushing_srv.h"

using namespace std;
using namespace pcl;
using namespace changkyu;

static MoveItNode* moveit_node;

static const string prefix_param("/changkyu/pkg_rss2019/demo_push_unknown_objects/");

#ifdef WIN32
  #define COLOR_NORMAL ""
  #define COLOR_RED ""
  #define COLOR_GREEN ""
  #define COLOR_YELLOW ""
  #define COLOR_CYAN ""
#else
  #define COLOR_NORMAL "\033[0m"
  #define COLOR_RED "\033[31m"
  #define COLOR_GREEN "\033[32m"
  #define COLOR_YELLOW "\033[33m"
  #define COLOR_CYAN "\033[36m"
#endif

static bool use_robot=true;
static bool use_pause=true;
static bool use_prop=true;

char WaitUserConfirm(const string str_message="[Enter]", const string str_done="[OK]", const string color=COLOR_RED)
{
    char ret;    
    if( use_pause )
    {
        std::cout << color << str_message << COLOR_NORMAL;
        //char ret = std::cin.get();
        std::cin >> ret;
        std::cin.ignore(INT_MAX,'\n');
        cout << COLOR_GREEN << str_done << COLOR_NORMAL << endl;
    }
    else
    {
        ros::Duration(1).sleep();
        ret = 0;
    }
    return ret;
}

string WaitUserInput(const string str_message="Input: ", const string str_done="[OK]")
{
    std::cout << COLOR_RED << str_message << COLOR_NORMAL;
    string ret;
    std::cin >> ret;
    std::cin.ignore(INT_MAX,'\n');
    cout << COLOR_GREEN << str_done << COLOR_NORMAL << endl;
    return ret;
}

typedef struct Marker
{
    int id;
    int x_px;
    int y_px;
    geometry_msgs::Pose pose;
} Marker;

bool WaitUserSelection(const std::vector<Marker> &markers_in, Marker &marker_select)
{
    if( markers_in.size()==1 )
    {
        marker_select = markers_in[0];
        if( marker_select.pose.position.z < 0.02 ) marker_select.pose.position.z = 0.02;
        return true;
    }
    else if( markers_in.size() > 1 )
    {
        while(true)
        {
            string str_id = WaitUserInput("Please enter the marker id: ");

            int id;
            stringstream ss;
            ss << str_id;
            ss >> id;
            
            for( int i=0; i<markers_in.size(); i++ )
            {
                if( markers_in[i].id==id )
                {
                    marker_select = markers_in[i];
                    if( marker_select.pose.position.z < 0.02 ) marker_select.pose.position.z = 0.02;
                    return true;
                }
            }
        }
    }
    return false;
}

void Quaterion2Euler(geometry_msgs::Quaternion &quat, double &angle_x, double &angle_y, double &angle_z)
{
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    double w = quat.w;
    
    double t0, t1, t2, t3, t4;

    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + y * y);

    t2 =  2.0 * (w * y - z * x);
    if( t2 >  1.0 ) t2 =  1.0;
    if( t2 < -1.0 ) t2 = -1.0;   

    t3 = 2.0 * (w * z + x * y);
    t4 = 1.0 - 2.0 * (y * y + z * z);

    angle_x = atan2(t0, t1);
    angle_y = asin(t2);
    angle_z = atan2(t3, t4);
}

typedef class MarkerManager
{
public:
    MarkerManager( ros::NodeHandle &nh, camerainfo_t &caminfo, double marker_size, 
                   double marker_resolution=5, double marker_margin=2 )
    : nh_(nh), caminfo_(caminfo)//marker_detector("ARUCO_MIP_36h12")
    {
        cv::Mat mx_cam = cv::Mat::eye(3,3,CV_32FC1);
        mx_cam.at<float>(0,0) = caminfo.camera_K[0];
        mx_cam.at<float>(0,2) = caminfo.camera_K[2];
        mx_cam.at<float>(1,1) = caminfo.camera_K[4];
        mx_cam.at<float>(1,2) = caminfo.camera_K[5];
        cam.setParams(mx_cam, cv::Mat::zeros(4,1,CV_32FC1), cv::Size(640,480));

        this->marker_size = marker_size;
    }

    void Detect(const cv::Mat &image, PointCloud<PointXYZRGB>::Ptr cloud, vector<Marker> &markers)
    {
        markers.clear();

        cout << "detect start" << endl;
        std::vector<aruco::Marker> dets = marker_detector.detect(image, cam, marker_size);
        cout << "detect done" << endl;
        
        Eigen::Matrix4f mx = changkyu::GetTransformCameraOnHand(nh_,caminfo_);
        for (size_t i=0; i<dets.size(); i++)
        {
            //Get the pose relative to the camera
            int id = dets[i].id;
            {
                double translation[3], quaternion[4];
                dets[i].OgreGetPoseParameters(translation, quaternion);
                double px = -translation[0];
                double py = -translation[1];
                double pz = translation[2];
                double qw = quaternion[0];
                double qx = quaternion[1];
                double qy = quaternion[2];
                double qz = quaternion[3];

                cout << translation[0] << ","
                     << translation[1] << ","
                     << translation[2] << endl;

                Eigen::Quaternionf q_cam(qw,qx,qy,qz);
                Eigen::Matrix3f rot;
                rot << mx(0,0),mx(0,1),mx(0,2), 
                       mx(1,0),mx(1,1),mx(1,2), 
                       mx(2,0),mx(2,1),mx(2,2);
                Eigen::Quaternionf q(rot);
                Eigen::Quaternionf q_world = q * q_cam;            
                
                double z = mx(3,0)*px + mx(3,1)*py + mx(3,2)*pz + mx(3,3);
                Marker marker;
                marker.id = id;
                marker.x_px = caminfo_.camera_K[0]*px/pz + caminfo_.camera_K[1]*py/pz + caminfo_.camera_K[2]; 
                marker.y_px = caminfo_.camera_K[3]*px/pz + caminfo_.camera_K[4]*py/pz + caminfo_.camera_K[5];
                marker.pose.orientation.x = q_world.x();
                marker.pose.orientation.y = q_world.y();
                marker.pose.orientation.z = q_world.z();
                marker.pose.orientation.w = q_world.w();
                marker.pose.position.x = (mx(0,0)*px + mx(0,1)*py + mx(0,2)*pz + mx(0,3))/z;
                marker.pose.position.y = (mx(1,0)*px + mx(1,1)*py + mx(1,2)*pz + mx(1,3))/z;
                if( cloud )
                    marker.pose.position.z = (*cloud)(marker.x_px,marker.y_px).z;
                else
                    //marker.pose.position.z = (mx(2,0)*px + mx(2,1)*py + mx(2,2)*pz + mx(2,3))/z;
                    marker.pose.position.z = 0.02;
                
                markers.push_back(marker);
            }
        }
    }

private:
    ros::NodeHandle nh_;
    camerainfo_t caminfo_;
    aruco::CameraParameters cam;
    aruco::MarkerDetector marker_detector;
    double marker_size;

} MarkerManager;

MarkerManager* marker_manager = NULL;

void GetRVizMarkers(vector<Marker> &markers, 
                    visualization_msgs::MarkerArray &markers_rviz,
                    double r=1,double g=0, double b=0)
{
    for( int i=0; i<markers.size(); i++ )
    {
        std::stringstream ss_txt;
        ss_txt << "[" << markers[i].id << "]";

        visualization_msgs::Marker marker_txt;
        marker_txt.header.frame_id = "/world";
        marker_txt.header.stamp = ros::Time::now();
        marker_txt.ns = ss_txt.str();
        marker_txt.id = 0;
        marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_txt.action = visualization_msgs::Marker::ADD;
        marker_txt.scale.x = 0.025;
        marker_txt.scale.y = 0.025;
        marker_txt.scale.z = 0.025;
        marker_txt.color.r = r;
        marker_txt.color.g = g;
        marker_txt.color.b = b;
        marker_txt.color.a = 1;
        marker_txt.pose.position.x = markers[i].pose.position.x;
        marker_txt.pose.position.y = markers[i].pose.position.y;
        marker_txt.pose.position.z = markers[i].pose.position.z + 0.10;
        marker_txt.pose.orientation.x = 0;
        marker_txt.pose.orientation.y = 0;
        marker_txt.pose.orientation.z = 0;
        marker_txt.pose.orientation.w = 1;
        marker_txt.text = ss_txt.str();
        marker_txt.lifetime = ros::Duration(10000);
        markers_rviz.markers.push_back(marker_txt);

        visualization_msgs::Marker marker_bar;
        marker_bar.header.frame_id = "/world";
        marker_bar.header.stamp = ros::Time::now();
        marker_bar.ns = ss_txt.str() + "_bar";
        marker_bar.id = 0;
        marker_bar.type = visualization_msgs::Marker::ARROW;
        marker_bar.action = visualization_msgs::Marker::ADD;
        marker_bar.scale.x = 0.0025;
        marker_bar.scale.y = 0.005;
        marker_bar.scale.z = 0.005;
        marker_bar.color.r = r;
        marker_bar.color.g = g;
        marker_bar.color.b = b;
        marker_bar.color.a = 1;
        marker_bar.pose.position.x = markers[i].pose.position.x;
        marker_bar.pose.position.y = markers[i].pose.position.y;
        marker_bar.pose.position.z = markers[i].pose.position.z;
        marker_bar.pose.orientation.x = 0;
        marker_bar.pose.orientation.y = 0;
        marker_bar.pose.orientation.z = 0;
        marker_bar.pose.orientation.w = 1;
        marker_bar.points.resize(2);
        marker_bar.points[0].x = 0;
        marker_bar.points[0].y = 0;
        marker_bar.points[0].z = 0.10;
        marker_bar.points[1].x = 0;
        marker_bar.points[1].y = 0;
        marker_bar.points[1].z = 0;
        marker_bar.lifetime = ros::Duration(10000);
        markers_rviz.markers.push_back(marker_bar);
    }
}

void GetMarkers(ros::NodeHandle &nh,
                cv::Mat &image,
                cv::Mat &depth,
                cv::Mat &mask_crop,
                camerainfo_t &caminfo,
                Eigen::Matrix4f &tf_cam2world,
                PointCloud<PointXYZRGB>::Ptr cloud_crop,
                vector<Marker> &markers,
                ros::Publisher* pub_cloud_rviz=NULL,
                ros::Publisher* pub_markers_rviz=NULL,
                double r=1, double g=0, double b=0 )
{
    bool repeat = false;
    do
    {
#if 1
        cv::Mat image_crop;
        PointCloud<PointXYZRGB>::Ptr cloud_all(new PointCloud<PointXYZRGB>);
        //PointCloud<PointXYZRGB>::Ptr cloud_crop(new PointCloud<PointXYZRGB>);
        GetInputFromCameraOnHand(nh, image, depth, cloud_all, caminfo, tf_cam2world,
                                 caminfo.workspace.size()>0, image_crop, mask_crop, cloud_crop);
#else
        PointCloud<PointXYZRGB>::Ptr cloud_all = NULL;
        sensor_msgs::Image::ConstPtr img_ptr, dep_ptr;
        sensor_msgs::CameraInfo::ConstPtr ci_depth;
        try
        {
            img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_image, nh, ros::Duration(10));
            dep_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_depth, nh);
            ci_depth = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo.topic_camerainfo,nh);
        }
        catch(std::exception &e)
        {
            ROS_ERROR("Exception during waitForMessage from camera: %s", e.what());
        }
        caminfo.camera_K.resize(9);
        for( int k=0; k<9; k++ ) caminfo.camera_K[k] = ci_depth->K[k];

        tf_cam2world = GetTransformCameraOnHand(nh,caminfo);

        cv_bridge::CvImagePtr cv_img
         = cv_bridge::toCvCopy(*img_ptr, sensor_msgs::image_encodings::RGB8);
        image = cv_img->image.clone();

        cv_bridge::CvImagePtr cv_dep
         = cv_bridge::toCvCopy(*dep_ptr, dep_ptr->encoding);
        depth = cv_dep->image.clone();

        cout << image.cols << endl;
        cout << depth.cols << endl;
#endif
        if( pub_cloud_rviz )
        {
            cloud_crop->header.frame_id = "/world";
            pub_cloud_rviz->publish(cloud_crop);
        }

        // Detect Marker for Estimate the Pose        
        marker_manager->Detect(image,cloud_all,markers);
        if( pub_markers_rviz )
        {
            visualization_msgs::MarkerArray markers_rviz;
            GetRVizMarkers(markers,markers_rviz,r,g,b);
            pub_markers_rviz->publish(markers_rviz);
            cout << "<< Markers >>" << endl;
            for( int i=0; i<markers.size(); i++ )
            {
                double roll, pitch, yaw;
                Quaterion2Euler(markers[i].pose.orientation, roll, pitch, yaw);

                cout << "[" << markers[i].id << "]: "
                     << "x: " << markers[i].pose.position.x << ","
                     << "y: " << markers[i].pose.position.y << ","
                     << "z: " << markers[i].pose.position.z << " "
                     << "r: " << roll  << ","
                     << "p: " << pitch << ","
                     << "y: " << yaw   << " "
                     << "pixel("
                     << markers[i].x_px << ","
                     << markers[i].y_px << ")" << endl;
            }
        }

        ros::spinOnce();

        if( markers.size()==0 )
            repeat = 'r'==WaitUserConfirm("[Markers] \'r\' for repeat, otherwise [Enter]");
        else
            repeat = false;

    } while( repeat );
}

void GetRVizPush(std::vector<geometry_msgs::Pose2D> &nexts,
                 double push0_x, double push0_y, double push0_z,
                 double push1_x, double push1_y, double push1_z,
                 visualization_msgs::MarkerArray &markers_rviz    )
{
    visualization_msgs::Marker marker_bar;
    marker_bar.header.frame_id = "/world";
    marker_bar.header.stamp = ros::Time::now();
    marker_bar.ns = "prepush";
    marker_bar.id = 0;
    marker_bar.type = visualization_msgs::Marker::ARROW;
    marker_bar.action = visualization_msgs::Marker::ADD;
    marker_bar.scale.x = 0.005;
    marker_bar.scale.y = 0.02;
    marker_bar.scale.z = 0.02;
    marker_bar.color.r = 1;
    marker_bar.color.g = 1;
    marker_bar.color.b = 0;
    marker_bar.color.a = 1;
    marker_bar.pose.position.x = push0_x;
    marker_bar.pose.position.y = push0_y;
    marker_bar.pose.position.z = push0_z;
    marker_bar.pose.orientation.x = 0;
    marker_bar.pose.orientation.y = 0;
    marker_bar.pose.orientation.z = 0;
    marker_bar.pose.orientation.w = 1;
    marker_bar.points.resize(2);
    marker_bar.points[0].x = 0;
    marker_bar.points[0].y = 0;
    marker_bar.points[0].z = 0.10;
    marker_bar.points[1].x = 0;
    marker_bar.points[1].y = 0;
    marker_bar.points[1].z = 0;
    marker_bar.lifetime = ros::Duration(10000);
    markers_rviz.markers.push_back(marker_bar);

    marker_bar.ns = "push";
    marker_bar.pose.position.x = push0_x;
    marker_bar.pose.position.y = push0_y;
    marker_bar.pose.position.z = push0_z;
    marker_bar.points[0].x = 0;
    marker_bar.points[0].y = 0;
    marker_bar.points[0].z = 0;
    marker_bar.points[1].x = push1_x-push0_x;
    marker_bar.points[1].y = push1_y-push0_y;
    marker_bar.points[1].z = push1_z-push0_z;
    markers_rviz.markers.push_back(marker_bar);

    marker_bar.ns = "postpush";
    marker_bar.pose.position.x = push1_x;
    marker_bar.pose.position.y = push1_y;
    marker_bar.pose.position.z = push1_z;
    marker_bar.points[0].x = 0;
    marker_bar.points[0].y = 0;
    marker_bar.points[0].z = 0;
    marker_bar.points[1].x = 0;
    marker_bar.points[1].y = 0;
    marker_bar.points[1].z = 0.10;
    markers_rviz.markers.push_back(marker_bar);

    marker_bar.ns = "path";
    marker_bar.id = 0;
    marker_bar.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_bar.scale.x = 0.005;
    marker_bar.scale.y = 0.02;
    marker_bar.scale.z = 0.02;
    marker_bar.color.r = 0;
    marker_bar.color.g = 0;
    marker_bar.color.b = 1;
    marker_bar.color.a = 1;
    marker_bar.points.resize(nexts.size());
    for( int n=0; n<nexts.size(); n++ )
    {        
        marker_bar.points[n].x = nexts[n].x;
        marker_bar.points[n].y = nexts[n].y;
        marker_bar.points[n].z = 0.10;
    }
    markers_rviz.markers.push_back(marker_bar);
}

void CenterOfPoints(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double &cx, double &cy)
{
    int n_pts = 0;
    cx = 0;
    cy = 0;
    for( int p=0; p<cloud.size(); p++ )
    {
        if( cloud[p].x!=0 && cloud[p].y!=0 && cloud[p].z!=0 )
        {
            cx += cloud[p].x;
            cy += cloud[p].y;
            n_pts++;
        }
    }
    cx /= n_pts;
    cy /= n_pts;
}

double compute_distance(double b, double a)
{
    double alpha = a * 180.0 / M_PI;
    double beta  = b * 180.0 / M_PI;
    int d = int(beta - alpha) % 360;
    double res;
    if( d > 180 )
        res = -360 + d;
    else if( d < -180 )
        res =  360 + d;
    else
        res = d;
    return res * M_PI / (double)180.0;
}

void Tracking( camerainfo_t &caminfo,
               const vector<Marker> &markers_0, 
               const vector<Marker> &markers_1,
               const Eigen::Matrix4f &tf_cam2world_0,
               const Eigen::Matrix4f &tf_cam2world_1,
               const vector<Marker> &pushing_0,
               vector<Marker> &pushing_1,
               geometry_msgs::Pose &tf_pose,
               ros::Publisher* pub_marker_rviz=NULL   )
{
    for( size_t i=0; i<markers_0.size(); i++ )
    {
        bool found = false;
        const Marker &marker_i = markers_0[i];
        for( size_t j=0; j<markers_1.size(); j++ )
        {            
            const Marker &marker_j = markers_1[j];
            if( marker_i.id == marker_j.id )
            {
                Eigen::Quaternionf q_i( marker_i.pose.orientation.w,
                                        marker_i.pose.orientation.x,
                                        marker_i.pose.orientation.y,
                                        marker_i.pose.orientation.z );
                Eigen::Quaternionf q_j( marker_j.pose.orientation.w,
                                        marker_j.pose.orientation.x,
                                        marker_j.pose.orientation.y,
                                        marker_j.pose.orientation.z );
                Eigen::Quaternionf q = q_i.inverse() * q_j;
                q.normalize();
                
                tf_pose.orientation.x = q.x();
                tf_pose.orientation.y = q.y();
                tf_pose.orientation.z = q.z();
                tf_pose.orientation.w = q.w();
                tf_pose.position.x = marker_j.pose.position.x - marker_i.pose.position.x;
                tf_pose.position.y = marker_j.pose.position.y - marker_i.pose.position.y;
                tf_pose.position.z = marker_j.pose.position.z - marker_i.pose.position.z;
                
                Eigen::Matrix3f rot = q.toRotationMatrix();
                for( int i=0; i<pushing_0.size(); i++ )        
                {
                    double x    = rot(0,0)*(pushing_0[i].pose.position.x - marker_i.pose.position.x) + 
                                  rot(0,1)*(pushing_0[i].pose.position.y - marker_i.pose.position.y) + 
                                  rot(0,2)*(pushing_0[i].pose.position.z - marker_i.pose.position.z) + 
                                  marker_j.pose.position.x;
                    double y    = rot(1,0)*(pushing_0[i].pose.position.x - marker_i.pose.position.x) + 
                                  rot(1,1)*(pushing_0[i].pose.position.y - marker_i.pose.position.y) + 
                                  rot(1,2)*(pushing_0[i].pose.position.z - marker_i.pose.position.z) + 
                                  marker_j.pose.position.y;
                    double z    = pushing_0[i].pose.position.z;

                    Eigen::Matrix4f tf_w2c = tf_cam2world_1.inverse();
                    double x_cam = tf_w2c(0,0)*x + 
                                   tf_w2c(0,1)*y + 
                                   tf_w2c(0,2)*z + 
                                   tf_w2c(0,3);
                    double y_cam = tf_w2c(1,0)*x + 
                                   tf_w2c(1,1)*y + 
                                   tf_w2c(1,2)*z + 
                                   tf_w2c(1,3);
                    double z_cam = tf_w2c(2,0)*x + 
                                   tf_w2c(2,1)*y + 
                                   tf_w2c(2,2)*z + 
                                   tf_w2c(2,3);
                    double x_px = (x_cam / z_cam)*caminfo.camera_K[0] + caminfo.camera_K[2];
                    double y_px = (y_cam / z_cam)*caminfo.camera_K[4] + caminfo.camera_K[5];

                    Marker marker;
                    marker = pushing_0[i];
                    marker.x_px = x_px;
                    marker.y_px = y_px;
                    marker.pose.position.x = x;
                    marker.pose.position.y = y;
                    marker.pose.position.z = z;
                    pushing_1.push_back(marker);
                }

                if( pub_marker_rviz )
                {
                    visualization_msgs::MarkerArray markers_rviz;
                    GetRVizMarkers(pushing_1,markers_rviz,1,0,0);
                    pub_marker_rviz->publish(markers_rviz);
                }
                return;
            }
        }
    }
}

void save(const string &fp_prefix, cv::Mat &image_begin, 
                                   cv::Mat &depth_begin, 
                                   cv::Mat &label_begin, 
                                   vector<Marker> &markers_begin,
                                   geometry_msgs::Pose tf_pose,
                                   cv::Mat &image_final,
                                   cv::Mat &depth_final,
                                   cv::Mat &label_final, 
                                   vector<Marker> &markers_final,
                                   int id_push, double push_time )
{
    char str_push[256];
    sprintf(str_push,"%s_push%d",fp_prefix.c_str(),id_push);

    cv::cvtColor(image_begin, image_begin, cv::COLOR_BGR2RGB);
    imwrite(string(str_push) + "_begin_color.png", image_begin);
    imwrite(string(str_push) + "_begin_depth.png", depth_begin);
    if( label_begin.cols > 0 && label_begin.rows )
        imwrite(string(str_push) + "_begin_label.png", label_begin);
    cv::cvtColor(image_final, image_final, cv::COLOR_BGR2RGB);
    imwrite(string(str_push) + "_final_color.png", image_final);
    imwrite(string(str_push) + "_final_depth.png", depth_final);
    if( label_final.cols > 0 && label_final.rows )
        imwrite(string(str_push) + "_final_label.png", label_final);

    ofstream ofs(string(str_push) + "_traj.txt");
    for( int i=0; i<markers_begin.size(); i++ )
    {
        if( markers_begin[i].id == id_push )
        {            
            ofs << "push:" << endl;
            ofs << " - id: " << id_push << endl;
            ofs << "   vec:" << endl;
            ofs << "     x: 0.10" << endl;
            ofs << "     y: 0.00" << endl;
            ofs << "     z: 0.00" << endl;
            ofs << "   x: "    << markers_begin[i].pose.position.x << endl;
            ofs << "   y: "    << markers_begin[i].pose.position.y << endl;
            ofs << "   z: "    << markers_begin[i].pose.position.z << endl;
            ofs << "   x_px: " << markers_begin[i].x_px << endl;
            ofs << "   y_px: " << markers_begin[i].y_px << endl;
            ofs << "   duration: " << push_time << endl;
            break;
        }
    }

    double roll, pitch, yaw;
    Quaterion2Euler(tf_pose.orientation, roll, pitch, yaw);
    ofs << "transform:" << endl;
    ofs << "  qx: " << tf_pose.orientation.x << endl;
    ofs << "  qy: " << tf_pose.orientation.y << endl;
    ofs << "  qz: " << tf_pose.orientation.z << endl;
    ofs << "  qw: " << tf_pose.orientation.w << endl;
    ofs << "  x: "  << tf_pose.position.x << endl;
    ofs << "  y: "  << tf_pose.position.y << endl;
    ofs << "  z: 0.0" << endl;
    ofs << "  roll: 0.0" << endl;
    ofs << "  pitch: 0.0" << endl;
    ofs << "  yaw: " << yaw << endl;

    ofs << "begin:" << endl;
    for( int i=0; i<markers_begin.size(); i++ )
    {
        if( 0 < markers_begin[i].id && markers_begin[i].id < 9 )
        {
            ofs << " - id: " << markers_begin[i].id << endl;
            ofs << "   qx: " << markers_begin[i].pose.orientation.x << endl;
            ofs << "   qy: " << markers_begin[i].pose.orientation.y << endl;
            ofs << "   qz: " << markers_begin[i].pose.orientation.z << endl;
            ofs << "   qw: " << markers_begin[i].pose.orientation.w << endl;
            ofs << "   x: "  << markers_begin[i].pose.position.x << endl;
            ofs << "   y: "  << markers_begin[i].pose.position.y << endl;
            ofs << "   z: "  << markers_begin[i].pose.position.z << endl;
            ofs << "   x_px: " << markers_begin[i].x_px << endl;
            ofs << "   y_px: " << markers_begin[i].y_px << endl;
        }
    }
    ofs << "final:" << endl;
    for( int i=0; i<markers_final.size(); i++ )
    {
        if( 0 < markers_final[i].id && markers_final[i].id < 9 )
        {
            ofs << " - id: " << markers_final[i].id << endl;
            ofs << "   qx: " << markers_final[i].pose.orientation.x << endl;
            ofs << "   qy: " << markers_final[i].pose.orientation.y << endl;
            ofs << "   qz: " << markers_final[i].pose.orientation.z << endl;
            ofs << "   qw: " << markers_final[i].pose.orientation.w << endl;
            ofs << "   x: "  << markers_final[i].pose.position.x << endl;
            ofs << "   y: "  << markers_final[i].pose.position.y << endl;
            ofs << "   z: "  << markers_final[i].pose.position.z << endl;
            ofs << "   x_px: " << markers_final[i].x_px << endl;
            ofs << "   y_px: " << markers_final[i].y_px << endl;
        }
    }
    ofs.close();
}

void Robot_GotoHome()
{
    std::vector<double> command_joint_position = {0, 0, 0, -90 * M_PI/180, 0, 90 * M_PI/180, 15 * M_PI/180};
    moveit_node->group.setJointValueTarget(command_joint_position);
    bool success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();
}

void Robot_GotoObservation()
{
    std::vector<double> command_joint_position = {0, 17 * M_PI/180, 0, -73 * M_PI/180, 0, 90 * M_PI/180, 15 * M_PI/180};
    moveit_node->group.setJointValueTarget(command_joint_position);
    bool success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();
}

void Robot_GotoGrasp()
{
    std::vector<double> command_joint_position = {-56*M_PI/180, 50*M_PI/180, 0, -60* M_PI/180, 0, 70*M_PI/180, -80*M_PI/180};
    //std::vector<double> command_joint_position = {56*M_PI/180, 50*M_PI/180, 0, -60* M_PI/180, 0, 70*M_PI/180, -50*M_PI/180};
    moveit_node->group.setJointValueTarget(command_joint_position);
    bool success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();

    ros::Duration(1).sleep();
    moveit_node->plan_and_execute_via_waypoints(0, 0, -0.15, 20);
}

void Robot_GotoGoalObservation()
{
    std::vector<double> command_joint_position = {-45*M_PI/180, 30*M_PI/180, 0, -40* M_PI/180, 0, 105*M_PI/180, 0};
    moveit_node->group.setJointValueTarget(command_joint_position);
    bool success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();

    ros::Duration(1).sleep();
    moveit_node->plan_and_execute_via_waypoints(0, 0, -0.15, 20);
}

void Robot_GotoPlace()
{
    std::vector<double> command_joint_position1 = {-56*M_PI/180,           0, 0, -90* M_PI/180, 0, 90*M_PI/180, -80*M_PI/180};    
    std::vector<double> command_joint_position2 = {           0,           0, 0, -90* M_PI/180, 0, 90*M_PI/180, -20*M_PI/180};    
    std::vector<double> command_joint_position3 = { 32*M_PI/180,           0, 0, -90* M_PI/180, 0, 90*M_PI/180,  20*M_PI/180};    
    std::vector<double> command_joint_position4 = { 32*M_PI/180, 45*M_PI/180, 0, -60* M_PI/180, 0, 90*M_PI/180,  20*M_PI/180};    
    
    bool success;
    moveit_node->group.setJointValueTarget(command_joint_position1);
    success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();
    //ros::Duration(1).sleep();
    //moveit_node->group.setJointValueTarget(command_joint_position2);
    //success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    //if (success) moveit_node->group.move();
    //ros::Duration(1).sleep();
    moveit_node->group.setJointValueTarget(command_joint_position3);
    success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();
    ros::Duration(1).sleep();
    moveit_node->group.setJointValueTarget(command_joint_position4);
    success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();
    ros::Duration(1).sleep();
}

void Robot_ChangeFinger(ros::Publisher &pub_gripper, char f)
{
    std_msgs::String msg;
    stringstream ss;
    ss << f;
    msg.data = ss.str();
    pub_gripper.publish(msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"changkyu_demo_push_unknown_objects");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::ServiceClient add_object    = nh.serviceClient<pkg_rss2019::add_object_srv   >("/pkg_rss2019/add_object",    true);
    ros::ServiceClient estimate_prop = nh.serviceClient<pkg_rss2019::estimate_prop_srv>("/pkg_rss2019/estimate_prop", true);
    ros::ServiceClient plan_pushing  = nh.serviceClient<pkg_rss2019::plan_prob_pushing_srv>("/pkg_rss2019/plan_prob_pushing", true);
    
    ros::Publisher pub_begin_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pkg_rss2019/markers/observation/begin", 100, true);
    ros::Publisher pub_final_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pkg_rss2019/markers/observation/final", 100, true);
    ros::Publisher pub_begin_marker = nh.advertise<visualization_msgs::MarkerArray>("/pkg_rss2019/markers/tags/begin", 100, true);
    ros::Publisher pub_final_marker = nh.advertise<visualization_msgs::MarkerArray>("/pkg_rss2019/markers/tags/final", 100, true);
    ros::Publisher pub_pushing      = nh.advertise<visualization_msgs::MarkerArray>("/pkg_rss2019/markers/pushing", 100, true);

    ros::Publisher pub_gripper = nh.advertise<std_msgs::String>("/GripperCommand", 1, true);

    nh.getParam(prefix_param + "use_robot",use_robot);
    nh.getParam(prefix_param + "use_pause",use_pause);
    nh.getParam(prefix_param + "use_prop",use_prop);
    ROS_INFO_STREAM("use_robot: " << use_robot);
    ROS_INFO_STREAM("use_pause: " << use_pause);
    ROS_INFO_STREAM("use_prop: " << use_prop);
    if( use_robot )
    {
        moveit_node = new MoveItNode;
        Robot_GotoHome();
        Robot_ChangeFinger(pub_gripper,'t');
    }

    geometry_msgs::Pose pose_observe;
    pose_observe.orientation.x = 0.130513665483;
    pose_observe.orientation.y = 0.991446495056;
    pose_observe.orientation.z = 0.0;
    pose_observe.orientation.w = 0.0;
    pose_observe.position.x = 0.500;
    pose_observe.position.y = 0.000;
    pose_observe.position.z = 0.750;

    geometry_msgs::Pose pose_push;
    pose_push.orientation.x = -0.60879131828;
    pose_push.orientation.y =  0.793330252171;
    pose_push.orientation.z = -0.000118264808139;
    pose_push.orientation.w = -0.000507681752219;
    pose_push.position.x = 0.500;
    pose_push.position.y = 0.000;
    pose_push.position.z = 0.400;

    Eigen::Quaternionf q_finger_push_x( -0.000507681752219,
                                        -0.608791318280000,
                                         0.793330252171000,
                                        -0.000118264808139 );

    Eigen::Quaternionf q_finger_bend_in(  0.9659258262890683, 
                                          0,
                                          0.25881904510252074,
                                          0 );

    Eigen::Quaternionf q_finger_bend_out( 0.9659258262890683, 
                                          0,
                                         -0.25881904510252074,
                                          0 );

    Eigen::Quaternionf q_finger_z180(0,0,0,1);

    // Get Parameters
    string fp_config, camera_name, object_name, dp_save, dp_objs;
    int id_push;
    double camera_offset_x, camera_offset_z, finger_length;
    vector<double> goal;
    nh.getParam(prefix_param + "camera_config",fp_config);
    nh.getParam(prefix_param + "camera_name",camera_name);
    nh.getParam(prefix_param + "camera_offset_x",camera_offset_x);
    nh.getParam(prefix_param + "camera_offset_z",camera_offset_z);
    nh.getParam(prefix_param + "object_name",object_name);
    nh.getParam(prefix_param + "objects_dir",dp_objs);
    nh.getParam(prefix_param + "push_point",id_push);
    nh.getParam(prefix_param + "finger_length",finger_length);    
    nh.getParam(prefix_param + "object_goal", goal);

    if( finger_length < 0.20 )
    {
        ROS_ERROR_STREAM("Please check the finger length param: " << finger_length << " (m)");
        return 0;    
    }

    map<string,camerainfo_t> name2camerainfo;
    changkyu::ParseParam(fp_config, name2camerainfo);
    camerainfo_t &caminfo = name2camerainfo.find(camera_name)->second;
    if( caminfo.camera_K.size()==0 )
        changkyu::UpdateIntrinsicParam(nh, caminfo);

    // Get Marker Info
    double marker_size;
    nh.getParam(prefix_param + "marker_size",marker_size);    
    marker_manager = new MarkerManager(nh,caminfo,marker_size);
    PointCloud<PointXYZRGB>::Ptr cloud_crop(new PointCloud<PointXYZRGB>);


#if 0
{
    int idx = 0;
    Robot_GotoGoalObservation();
    while( true )
    {
        cv::Mat image_0, depth_0, label_0;
        vector<Marker> markers_0;
        Marker marker_0;
        Eigen::Matrix4f tf_cam2world_0;
        char fp_image[256];
        char fp_pose[256];
        double roll, pitch, yaw;

        sprintf(fp_pose,"/home/cm1074/changkyu/iros2020/dataset/goals/%s_%d_pose.txt",object_name.c_str(),idx);
        ofstream of(fp_pose);

        WaitUserConfirm("Take Before");
        GetMarkers(nh,image_0,depth_0,label_0,caminfo,tf_cam2world_0,cloud_crop,markers_0,
                   &pub_begin_cloud, &pub_begin_marker,1,1,0 );
        WaitUserSelection(markers_0, marker_0);
        
        cv::cvtColor(image_0, image_0, cv::COLOR_BGR2RGB);        
        sprintf(fp_image,"/home/cm1074/changkyu/iros2020/dataset/goals/%s_%d_begin.png",object_name.c_str(),idx);
        imwrite(fp_image, image_0);
        Quaterion2Euler(marker_0.pose.orientation, roll, pitch, yaw);

        of << "before:" << std::endl
           << "  x: " << marker_0.pose.position.x << std::endl
           << "  y: " << marker_0.pose.position.y << std::endl
           << "  z: " << marker_0.pose.position.z << std::endl
           << "  qx: " << marker_0.pose.orientation.x << std::endl
           << "  qy: " << marker_0.pose.orientation.y << std::endl
           << "  qz: " << marker_0.pose.orientation.z << std::endl
           << "  qw: " << marker_0.pose.orientation.w << std::endl
           << "  roll: " << roll << std::endl
           << "  pitch: " << pitch << std::endl
           << "  yaw: " << yaw << std::endl;

        WaitUserConfirm("Take After");
        GetMarkers(nh,image_0,depth_0,label_0,caminfo,tf_cam2world_0,cloud_crop,markers_0,
                   &pub_begin_cloud, &pub_begin_marker,1,1,0 );
        if( WaitUserSelection(markers_0, marker_0)==false )
        {
            marker_0.pose.position.x = -1;
            marker_0.pose.position.y = -1;
            marker_0.pose.position.z = -1;
            marker_0.pose.orientation.x = -1;
            marker_0.pose.orientation.y = -1;
            marker_0.pose.orientation.z = -1;
            marker_0.pose.orientation.w = -1;
            roll = -1;
            pitch = -1;
            yaw = -1;
        }
        else
        {
            Quaterion2Euler(marker_0.pose.orientation, roll, pitch, yaw);
        }

        cv::cvtColor(image_0, image_0, cv::COLOR_BGR2RGB);
        sprintf(fp_image,"/home/cm1074/changkyu/iros2020/dataset/goals/%s_%d_final.png",object_name.c_str(),idx);
        imwrite(fp_image, image_0);

        of << "after:" << std::endl
           << "  x: " << marker_0.pose.position.x << std::endl
           << "  y: " << marker_0.pose.position.y << std::endl
           << "  z: " << marker_0.pose.position.z << std::endl
           << "  qx: " << marker_0.pose.orientation.x << std::endl
           << "  qy: " << marker_0.pose.orientation.y << std::endl
           << "  qz: " << marker_0.pose.orientation.z << std::endl
           << "  qw: " << marker_0.pose.orientation.w << std::endl
           << "  roll: " << roll << std::endl
           << "  pitch: " << pitch << std::endl
           << "  yaw: " << yaw << std::endl;

        of.close();

        idx++;
    }
}
#endif


    ROS_INFO("[Planner] add the object to planner");
    {
        string fp_image = dp_objs + "/" + object_name + "_label.pgm";
        cout << fp_image << endl;
        cv::Mat img = cv::imread(fp_image, cv::IMREAD_UNCHANGED);

        cv_bridge::CvImage cv_image;
        sensor_msgs::Image msg_img;
        cv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        cv_image.image    = img;
        cv_image.toImageMsg(msg_img);

        pkg_rss2019::add_object_srv objinfo;
        objinfo.request.name = object_name;
        objinfo.request.image = msg_img;
        objinfo.request.meter_per_pixel = 0.53 / 640.0;
        add_object.call(objinfo);
    }
    ROS_INFO("[Planner] add the object to planner - done");

#if 0
    if( use_robot )
    {
        // Move to observed mode
        ROS_INFO("[Move] observation pose");
        Robot_GotoObservation();
        ROS_INFO("[Move] observation pose - done");
    }

    // Get observation - BEGIN    
    ROS_INFO("[Camera] sleep 2 secs");
    ros::Duration(2).sleep();
    ROS_INFO("[Camera] sleep 2 secs - done");

    WaitUserConfirm("[!] Place the object to the goal configuration [!]");    
    ROS_INFO("[Camera] detect markers");
    cv::Mat image_1, depth_1, label_1;
    vector<Marker> markers_1;
    Eigen::Matrix4f tf_cam2world_1;
    GetMarkers(nh,image_1,depth_1,label_1,caminfo,tf_cam2world_1,cloud_crop,markers_1,
               &pub_final_cloud, &pub_final_marker,0,0,1);
    ROS_INFO("[Camera] detect markers - done");
    Marker marker_1;
    WaitUserSelection(markers_1, marker_1);
    cout << marker_1.pose << endl;
#else

    ROS_INFO_STREAM("Goal State: " << goal[0] << "," << goal[1] << "," << goal[2]);

    Eigen::Quaternionf q_goal;
    q_goal = Eigen::AngleAxisf(goal[2]/180.0*M_PI, Eigen::Vector3f::UnitZ());
    Marker marker_1;
    marker_1.pose.position.x = goal[0];
    marker_1.pose.position.y = goal[1];
    marker_1.pose.position.z = 0.05;
    marker_1.pose.orientation.w = q_goal.w();
    marker_1.pose.orientation.x = q_goal.x();
    marker_1.pose.orientation.y = q_goal.y();
    marker_1.pose.orientation.z = q_goal.z();

#endif

    WaitUserConfirm("[!] Place the object to the init configuration [!]");

    Robot_GotoObservation();
    
    ROS_INFO("[Camera] detect markers");
    cv::Mat image_0, depth_0, label_0;
    vector<Marker> markers_0;
    Eigen::Matrix4f tf_cam2world_0;

    GetMarkers(nh,image_0,depth_0,label_0,caminfo,tf_cam2world_0,cloud_crop,markers_0,
               &pub_begin_cloud, &pub_begin_marker,1,1,0 );
    ROS_INFO("[Camera] detect markers - done");
    Marker marker_0, marker_0_prv;
    WaitUserSelection(markers_0, marker_0);
    //marker_0.pose.position.z += 0.20;

    pkg_rss2019::estimate_prop_srv observation;
    observation.request.name = object_name;

    double offset_m2cx, offset_m2cy;
    bool is_first = true;
    double yaw_goal;
    while(true)
    {
        ROS_INFO("[Move] observation pose");
        pose_observe.position.x = marker_0.pose.position.x - camera_offset_x;
        pose_observe.position.y = marker_0.pose.position.y;
        pose_observe.position.z = marker_0.pose.position.z + 0.400 - camera_offset_z;
        //pose_observe.position.z = marker_0.pose.position.z + 0.200 - camera_offset_z;
        if( is_first )
        {
            //pose_observe.position.z += 0.2;
            moveit_node->plan_and_execute(pose_observe);
        }
        else
        {
            moveit_node->plan_and_execute_via_waypoints(pose_observe);
        }
        ROS_INFO("[Move] observation pose - done");

        ROS_INFO("[Camera] detect markers");
        GetMarkers(nh,image_0,depth_0,label_0,caminfo,tf_cam2world_0,cloud_crop,markers_0,
                   &pub_begin_cloud, &pub_begin_marker,1,1,0 );
        ROS_INFO("[Camera] detect markers - done");
        WaitUserSelection(markers_0, marker_0);

        //marker_0.pose.position.z += 0.20;

        ROS_INFO("[Camera] compute meter_per_pixel");
        geometry_msgs::PoseStamped::ConstPtr pose_ee_cur
         = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
            "/iiwa/state/CartesianPose",nh,ros::Duration(10));
        double meter_per_pixel = (pose_ee_cur->pose.position.z + camera_offset_z)/(float)640.0;        
        ROS_INFO_STREAM("[Camera] compute meter_per_pixel: " << meter_per_pixel);

        ROS_INFO("[Planner] check if it achieved the goal");
        double roll, pitch, yaw0, yaw1;
        Quaterion2Euler(marker_0.pose.orientation, roll, pitch, yaw0);
        Quaterion2Euler(marker_1.pose.orientation, roll, pitch, yaw1);
        //yaw0 -= M_PI * 0.5; // correction due to the coordinate difference
        //yaw1 -= M_PI * 0.5;
        yaw0 = compute_distance(yaw0, M_PI*0.5);
        yaw1 = compute_distance(yaw1, M_PI*0.5);

        if( is_first )
        {
#if 0
            double dist1 = abs(compute_distance(yaw0, yaw1));
            double dist2 = abs(compute_distance(yaw0, yaw1+M_PI));
            yaw_goal = dist1 < dist2 ? yaw1 : yaw1+M_PI;
            yaw_goal = compute_distance(yaw_goal,0);
            ROS_INFO_STREAM(COLOR_GREEN << "[Select Goal] " 
                                        << marker_1.pose.position.x << ","
                                        << marker_1.pose.position.y << ","
                                        << yaw_goal * 180.0 / M_PI );
#else
            yaw_goal = yaw1;
#endif
        }

        double err_xy = sqrt((marker_1.pose.position.x - marker_0.pose.position.x)*
                             (marker_1.pose.position.x - marker_0.pose.position.x)+
                             (marker_1.pose.position.y - marker_0.pose.position.y)*
                             (marker_1.pose.position.y - marker_0.pose.position.y));
        double err_yaw = compute_distance(yaw_goal, yaw0);
        bool success = (err_xy <= 0.07) && (abs(err_yaw) <= 15.0/180.0*M_PI);
        cout << COLOR_GREEN << "[CHECK ERROR] xy: " << err_xy << ", yaw: " << err_yaw/M_PI*180.0 << COLOR_NORMAL << endl;
        ROS_INFO_STREAM("[Planner] check if it achieved the goal - " << (success ? "SUCCESS" : "NOT YET"));
        if( success ) break;

        cv::Mat img;
        cv_bridge::CvImage cv_image;
        if( is_first )
        {
            double cx, cy;
            CenterOfPoints(*cloud_crop, cx, cy);
            double x_tmp = cx - marker_0.pose.position.x;
            double y_tmp = cy - marker_0.pose.position.y;
            //offset_m2cx = cos(-yaw0)*x_tmp-sin(-yaw0)*y_tmp;
            //offset_m2cy = sin(-yaw0)*x_tmp+cos(-yaw0)*y_tmp;
            offset_m2cx = 0; 
            offset_m2cy = 0.05;

            cout << "offset org: " << offset_m2cx << ", " << offset_m2cy << endl;

            /*            
            cv::Point2f pt(label_0.cols*0.5,label_0.rows*0.5);
            cv::Mat rot = cv::getRotationMatrix2D(pt, -yaw0 / M_PI * 180.0, 1.0);
            cv::warpAffine(label_0,img,rot,label_0.size(),cv::INTER_NEAREST);
            img = img / 255;

            cv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            cv_image.image    = img;
            cv_image.toImageMsg(msg_img);
            */
            is_first = false;
        }
        else
        {
            if(use_prop)
            {
                ROS_INFO("[Planner] estimate object properties");
                double x_tmp = cos(yaw0)*offset_m2cx-sin(yaw0)*offset_m2cy;
                double y_tmp = sin(yaw0)*offset_m2cx+cos(yaw0)*offset_m2cy;

                cout << "offset rot: " << x_tmp << ", " << y_tmp << endl;

                observation.request.pos_final.x = marker_0.pose.position.x + x_tmp;
                observation.request.pos_final.y = marker_0.pose.position.y + y_tmp;
                estimate_prop.call(observation);
                ROS_INFO("[Planner] estimate object properties - done");
            }
        }
        observation.request.pos_begin.y = observation.request.pos_final.x;
        observation.request.pos_begin.y = observation.request.pos_final.y;

        ROS_INFO("[Planner] query a plan");

        double x_tmp0 = cos(yaw0)*offset_m2cx-sin(yaw0)*offset_m2cy;
        double y_tmp0 = sin(yaw0)*offset_m2cx+cos(yaw0)*offset_m2cy;

        pkg_rss2019::plan_prob_pushing_srv plan;
        plan.request.name = object_name;
        plan.request.init.x = marker_0.pose.position.x + x_tmp0;
        plan.request.init.y = marker_0.pose.position.y + y_tmp0;
        plan.request.init.x = marker_0.pose.position.x;
        plan.request.init.y = marker_0.pose.position.y;
        plan.request.init.theta = yaw0;
#if 0        
        plan.request.goals.resize(2);
        plan.request.goals[0].x = marker_1.pose.position.x;
        plan.request.goals[0].y = marker_1.pose.position.y;
        plan.request.goals[0].theta = yaw1;
        plan.request.goals[1].x = marker_1.pose.position.x;
        plan.request.goals[1].y = marker_1.pose.position.y;
        plan.request.goals[1].theta = yaw1 + M_PI;
#else
        plan.request.goals.resize(1);
        plan.request.goals[0].x = marker_1.pose.position.x;
        plan.request.goals[0].y = marker_1.pose.position.y;
        plan.request.goals[0].theta = yaw_goal;
#endif        
        for(int g=0; g<plan.request.goals.size(); g++ )
        {
            double yaw = plan.request.goals[g].theta;
            double x_tmp1 = cos(yaw)*offset_m2cx-sin(yaw)*offset_m2cy;
            double y_tmp1 = sin(yaw)*offset_m2cx+cos(yaw)*offset_m2cy;
            plan.request.goals[g].x += x_tmp1;
            plan.request.goals[g].y += y_tmp1;
        }
        plan_pushing.call(plan);
        ROS_INFO("[Planner] query a plan - done");

        int n_pushes = plan.response.pushes_beg.size();
        if( n_pushes<1 ) break;

        for( int p=0; p<n_pushes; p++ )
        {
            double push0_x = plan.response.pushes_beg[p].x + x_tmp0;
            double push0_y = plan.response.pushes_beg[p].y + y_tmp0;
            double push1_x = plan.response.pushes_end[p].x + x_tmp0;
            double push1_y = plan.response.pushes_end[p].y + y_tmp0;
            double vec_push_x = push1_x-push0_x;
            double vec_push_y = push1_y-push0_y;
            double len_push = sqrt(vec_push_x*vec_push_x + vec_push_y*vec_push_y);        
            double uvec_push_x = vec_push_x / len_push;
            double uvec_push_y = vec_push_y / len_push;
            
            double finger_yaw = uvec_push_x >= 0 ? asin(uvec_push_y) : asin(-uvec_push_y);
            Eigen::Quaternionf q_finger_push;
            q_finger_push = Eigen::AngleAxisf(finger_yaw, Eigen::Vector3f::UnitZ());

            Eigen::Quaternionf q_finger;
            double dist0 = sqrt(push0_x*push0_x + push0_y*push0_y);
            double dist1 = sqrt(push1_x*push1_x + push1_y*push1_y);
            if( (abs(uvec_push_x)>=abs(uvec_push_y)*0.5) && (dist0 < 0.40 || dist1 < 0.45) )
            {
                cout << "bend in" << endl;
                if( uvec_push_x >= 0 )
                {
                    q_finger = q_finger_push * q_finger_bend_in * q_finger_push_x;
                    pose_push.position.x = push0_x + (-0.00 + 0.02*sqrt(3)*0.5 + finger_length*0.5)*uvec_push_x;
                    pose_push.position.y = push0_y + (-0.00 + 0.02*sqrt(3)*0.5 + finger_length*0.5)*uvec_push_y;
                    pose_push.position.z = marker_0.pose.position.z + (finger_length*sqrt(3)*0.5 - 0.02*0.5);
                }
                else
                {
                    q_finger = q_finger_push * q_finger_bend_in * q_finger_z180 * q_finger_push_x;
                    pose_push.position.x = push0_x + (-0.00 + 0.02*sqrt(3)*0.5 - finger_length*0.5)*uvec_push_x;
                    pose_push.position.y = push0_y + (-0.00 + 0.02*sqrt(3)*0.5 - finger_length*0.5)*uvec_push_y;
                    pose_push.position.z = marker_0.pose.position.z + (finger_length*sqrt(3)*0.5 + 0.02*0.5);
                }
            }
            else if( (abs(uvec_push_x)>=abs(uvec_push_y)*0.5) && (dist0 > 0.72 || dist1 > 0.72) )
            {
                cout << "bend out" << endl;
                if( uvec_push_x >= 0 )
                {
                    q_finger = q_finger_push * q_finger_bend_out * q_finger_push_x;
                    pose_push.position.x = push0_x + (-0.00 + 0.02*sqrt(3)*0.5 - finger_length*0.5)*uvec_push_x;
                    pose_push.position.y = push0_y + (-0.00 + 0.02*sqrt(3)*0.5 - finger_length*0.5)*uvec_push_y;
                    pose_push.position.z = marker_0.pose.position.z + (finger_length*sqrt(3)*0.5 + 0.02*0.5);
                }
                else
                {
                    q_finger = q_finger_push * q_finger_bend_out * q_finger_z180 * q_finger_push_x;
                    pose_push.position.x = push0_x + (-0.00 + 0.02*sqrt(3)*0.5 + finger_length*0.5)*uvec_push_x;
                    pose_push.position.y = push0_y + (-0.00 + 0.02*sqrt(3)*0.5 + finger_length*0.5)*uvec_push_y;
                    pose_push.position.z = marker_0.pose.position.z + (finger_length*sqrt(3)*0.5 - 0.02*0.5);
                }
            }
            else
            {
                if( uvec_push_x >= 0 )
                    q_finger = q_finger_push * q_finger_push_x;
                else
                    q_finger = q_finger_push * q_finger_z180 * q_finger_push_x;

                pose_push.position.x = push0_x + (-0.00 + 0.02)*uvec_push_x;
                pose_push.position.y = push0_y + (-0.00 + 0.02)*uvec_push_y; // finger offset
                pose_push.position.z = marker_0.pose.position.z + finger_length;
            }
            
            pose_push.orientation.x = q_finger.x();
            pose_push.orientation.y = q_finger.y();
            pose_push.orientation.z = q_finger.z();
            pose_push.orientation.w = q_finger.w();

            visualization_msgs::MarkerArray markers_rviz_push;
            GetRVizPush( plan.response.nexts,
                         push0_x, push0_y, marker_0.pose.position.z,
                         push1_x, push1_y, marker_0.pose.position.z,
                         markers_rviz_push );
            pub_pushing.publish(markers_rviz_push);

			ros::Time time_push_beg = ros::Time::now();
            ROS_INFO_STREAM("[Plan] push");
            bool succ = moveit_node->plan_and_execute_U_push(pose_push, 
                                                             uvec_push_x*(len_push+0.005), 
                                                             uvec_push_y*(len_push+0.005), 0.10);

            if( succ )
            {
                ROS_INFO_STREAM("[Plan] push - done");
                double yaw = plan.response.nexts[p].theta;
                double x_tmp = cos(yaw)*offset_m2cx-sin(yaw)*offset_m2cy;
                double y_tmp = sin(yaw)*offset_m2cx+cos(yaw)*offset_m2cy;

                marker_0.pose.position.x = plan.response.nexts[p].x - x_tmp;
                marker_0.pose.position.y = plan.response.nexts[p].y - y_tmp;
                ROS_INFO_STREAM("[Plan] predicted pos: " << marker_0.pose.position.x << ","
                                                         << marker_0.pose.position.y        );

                ros::Time time_push_end = ros::Time::now();
                double time_push = (time_push_end-time_push_beg).toSec();

                observation.request.pos_push.x = push0_x;
                observation.request.pos_push.y = push0_y;
                observation.request.vec_push.x = vec_push_x;
                observation.request.vec_push.y = vec_push_y;
                observation.request.duration = time_push;

                break;
            }
            else
            {
                ROS_ERROR_STREAM("[Plan] push - failed (try another plan)");
            }
        }
    }

    ROS_INFO("[FINISHED]");

    if( use_robot )
    {
        if( 'g'==WaitUserConfirm("[Graps] \'g\' for grasp, otherwise [Enter]", "[OK]", COLOR_CYAN) )
        {
            Robot_ChangeFinger(pub_gripper,'o');
            ros::Duration(3).sleep();
    
            Robot_GotoGrasp();
            WaitUserConfirm("[Grasp] Are you ready?");
            Robot_ChangeFinger(pub_gripper,'g');
            ros::Duration(2).sleep();
            moveit_node->plan_and_execute_via_waypoints(0, 0, 0.10, 20);
            Robot_GotoPlace();
            Robot_ChangeFinger(pub_gripper,'o');
            ros::Duration(2).sleep();
            moveit_node->plan_and_execute_via_waypoints(0, 0, 0.10, 20);

            WaitUserConfirm("[FINISHED] Goto Home? [Enter]");
            Robot_ChangeFinger(pub_gripper,'o');
            moveit_node->plan_and_execute_via_waypoints(0, 0, 0.10, 20);
            ros::Duration(1).sleep();
            Robot_GotoHome();
        }
        else
        {
            Robot_GotoHome();
        }
    }
    
    bool running = true;
    ros::Rate r(30);
    while( ros::ok() && running )
    {
        ros::spinOnce();
    }
    ros::shutdown();

    if( marker_manager ) delete marker_manager;

    return 0;
}
