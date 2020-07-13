#include <math.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
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

#include <aruco.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "camera/camera_config.hpp"
#include "camera/camera_utils.hpp"

using namespace std;
using namespace pcl;
using namespace changkyu;

static MoveItNode* moveit_node;

static const string prefix_param("/changkyu/pkg_rss2019/demo_push_objects/");

#ifdef WIN32
  #define COLOR_NORMAL ""
  #define COLOR_RED ""
  #define COLOR_GREEN ""
  #define COLOR_YELLOW ""
#else
  #define COLOR_NORMAL "\033[0m"
  #define COLOR_RED "\033[31m"
  #define COLOR_GREEN "\033[32m"
  #define COLOR_YELLOW "\033[33m"
#endif

static bool use_robot=true;
static bool use_pause=true;

char WaitUserConfirm(const string str_message="[Enter]", const string str_done="[OK]")
{
    char ret;    
    if( use_pause )
    {
        std::cout << COLOR_RED << str_message << COLOR_NORMAL;
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

typedef class MarkerManager
{
public:
    MarkerManager( ros::NodeHandle &nh, camerainfo_t &caminfo, double marker_size, 
                   double marker_resolution=5, double marker_margin=2 )
    : nh_(nh), caminfo_(caminfo)//, marker_detector("ARUCO_MIP_36h12")
    {
        cv::Mat mx_cam = cv::Mat::eye(3,3,CV_32FC1);
        mx_cam.at<float>(0,0) = caminfo.camera_K[0];
        mx_cam.at<float>(0,2) = caminfo.camera_K[2];
        mx_cam.at<float>(1,1) = caminfo.camera_K[4];
        mx_cam.at<float>(1,2) = caminfo.camera_K[5];
        cam.setParams(mx_cam, cv::Mat::zeros(4,1,CV_32FC1), cv::Size(640,480));

        this->marker_size = marker_size;
    }
    
    void GenPushingPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          vector<Marker> &markers)
    {
        markers.clear();

        static const int n_points = 10;

        // from south -> north
        vector<Marker> markers_all;
        int id, interval, i_beg, i_end;

        for( int c=0; c<cloud->width; c++ )
        {
            for( int r=cloud->height-1; r>=0; r-- )
            {
                pcl::PointXYZRGB &pt = (*cloud)(c,r);
                if( 0.02 < pt.z && pt.z < 0.10 ) // reasonable position
                {
                    Marker marker;
                    marker.id = 0;
                    marker.x_px = c;
                    marker.y_px = r;
                    marker.pose.orientation.x = 0;
                    marker.pose.orientation.y = 0;
                    marker.pose.orientation.z = 0;
                    marker.pose.orientation.w = 1;
                    marker.pose.position.x = pt.x;
                    marker.pose.position.y = pt.y;
                    marker.pose.position.z = pt.z;
                    if( marker.pose.position.z < 0.03 ) 
                        marker.pose.position.z = 0.03;
                    markers_all.push_back(marker);
                    break;
                }
            } 
        }

        id = 1;
        interval = markers_all.size() / (n_points+1);
        i_beg = markers_all.size()/2-(n_points-1)/2*interval+interval;
        i_end = markers_all.size()/2+(n_points-1)/2*interval-interval;
        for( int i=i_beg; i<=i_end; i=i+interval )
        {
            Marker &marker = markers_all[i];
            marker.id = id;            
            markers.push_back(marker);
            id++;
        }

        markers_all.clear();
        for( int c=0; c<cloud->width; c++ )
        {
            for( int r=0; r<cloud->height; r++ )
            {
                pcl::PointXYZRGB &pt = (*cloud)(c,r);
                if( 0.02 < pt.z && pt.z < 0.10 ) // reasonable position
                {
                    Marker marker;
                    marker.id = 0;
                    marker.x_px = c;
                    marker.y_px = r;
                    marker.pose.orientation.x = 0;
                    marker.pose.orientation.y = 0;
                    marker.pose.orientation.z = 0;
                    marker.pose.orientation.w = 1;
                    marker.pose.position.x = pt.x;
                    marker.pose.position.y = pt.y;
                    marker.pose.position.z = pt.z;
                    if( marker.pose.position.z < 0.03 ) 
                        marker.pose.position.z = 0.03;
                    markers_all.push_back(marker);
                    break;
                }
            } 
        }

        interval = markers_all.size() / (n_points+1);
        i_beg = markers_all.size()/2-(n_points-1)/2*interval+interval;
        i_end = markers_all.size()/2+(n_points-1)/2*interval-interval;
        for( int i=i_beg; i<=i_end; i=i+interval )
        {
            Marker &marker = markers_all[i];
            marker.id = id;            
            markers.push_back(marker);
            id++;
        }
    }

    void Detect(const cv::Mat &image, PointCloud<PointXYZRGB>::Ptr cloud, vector<Marker> &markers)
    {
        int n_pts=0;
        double cx=0, cy=0;
        for( int p=0; p<cloud->size(); p++ )
        {
            if( cloud->points[p].z > 0.01 && cloud->points[p].z < 0.20 )
            {
                cx += cloud->points[p].x;
                cy += cloud->points[p].y;
                n_pts++;
            }
        }
        cx /= (double)n_pts;
        cy /= (double)n_pts;

        markers.clear();
        std::vector<aruco::Marker> dets = marker_detector.detect(image, cam, marker_size);
        
        Eigen::Matrix4f mx = changkyu::GetTransformCameraOnHand(nh_,caminfo_);
        for (size_t i=0; i<dets.size(); i++)
        {
            //Get the pose relative to the camera
            int id = dets[i].id;
            //if( 0 <= id && id <= 9 )
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
                //marker.pose.position.z = (mx(2,0)*px + mx(2,1)*py + mx(2,2)*pz + mx(2,3))/z;
                marker.pose.position.z = (*cloud)(marker.x_px,marker.y_px).z;
                markers.push_back(marker);

                cout << "mx: " << mx << endl;

                cout << "offset marker to com: " << endl;
                cout << cx - marker.pose.position.x << endl;
                cout << cy - marker.pose.position.y << endl;
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
                vector<Marker> &markers,
                ros::Publisher* pub_cloud_rviz=NULL,
                ros::Publisher* pub_markers_rviz=NULL,
                double r=1, double g=0, double b=0,
                vector<Marker>* markers_pushing=NULL,
                ros::Publisher* pub_pushing_rviz=NULL,
                double r_pushing=1, double g_pushing=1, double b_pushing=1 )
{
    bool repeat = false;
    do
    {
        cv::Mat image_crop;
        PointCloud<PointXYZRGB>::Ptr cloud_all(new PointCloud<PointXYZRGB>);
        PointCloud<PointXYZRGB>::Ptr cloud_crop(new PointCloud<PointXYZRGB>);
        GetInputFromCameraOnHand(nh, image, depth, cloud_all, caminfo, tf_cam2world,
                                 caminfo.workspace.size()>0, image_crop, mask_crop, cloud_crop);

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
                cout << "[" << markers[i].id << "]: "
                     << markers[i].pose.position.x << ","
                     << markers[i].pose.position.y << ","
                     << markers[i].pose.position.z << " pixel("
                     << markers[i].x_px << ","
                     << markers[i].y_px << ")" << endl;
            }
        }

        // Generate possible pushing points
        if( markers_pushing )
        {
            marker_manager->GenPushingPoints(cloud_crop,*markers_pushing);
            if( pub_pushing_rviz )
            {
                visualization_msgs::MarkerArray markers_rviz;
                GetRVizMarkers(*markers_pushing,markers_rviz,r_pushing,g_pushing,b_pushing);
                pub_pushing_rviz->publish(markers_rviz);
                cout << "<< Pushing Points >>" << endl;
                for( int i=0; i<markers_pushing->size(); i++ )
                {
                    cout << "[" << (*markers_pushing)[i].id << "]: "
                         << (*markers_pushing)[i].pose.position.x << ","
                         << (*markers_pushing)[i].pose.position.y << ","
                         << (*markers_pushing)[i].pose.position.z << " pixel("
                         << (*markers_pushing)[i].x_px << ","
                         << (*markers_pushing)[i].y_px << ")" << endl;
                }
            }
        } 

        ros::spinOnce();

        char ret = WaitUserConfirm("[Markers] \'r\' for repeat, otherwise [Enter]");
        if( ret=='r' ) repeat = true;
        else           repeat = false;
    } while( repeat );
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
                                   int id_push, double push_x, double push_time )
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
            ofs << "     x: " << push_x << endl;
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
    std::vector<double> command_joint_position = {0, 17 * M_PI/180, 0, -52 * M_PI/180, 0, 110 * M_PI/180, 15 * M_PI/180};
    moveit_node->group.setJointValueTarget(command_joint_position);
    bool success = (bool)moveit_node->group.plan(moveit_node->my_plan);
    if (success) moveit_node->group.move();
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"changkyu_demo_push_objects");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Publisher pub_begin_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pkg_rss2019/markers/observation/begin", 100, true);
    ros::Publisher pub_final_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pkg_rss2019/markers/observation/final", 100, true);
    ros::Publisher pub_begin_marker = nh.advertise<visualization_msgs::MarkerArray>("/pkg_rss2019/markers/tags/begin", 100, true);
    ros::Publisher pub_final_marker = nh.advertise<visualization_msgs::MarkerArray>("/pkg_rss2019/markers/tags/final", 100, true);
    ros::Publisher pub_begin_push = nh.advertise<visualization_msgs::MarkerArray>("/pkg_rss2019/markers/pushing/begin", 100, true);
    ros::Publisher pub_final_push = nh.advertise<visualization_msgs::MarkerArray>("/pkg_rss2019/markers/pushing/final", 100, true);

    nh.getParam(prefix_param + "use_robot",use_robot);
    nh.getParam(prefix_param + "use_pause",use_pause);
    ROS_INFO_STREAM("use_robot: " << use_robot);
    ROS_INFO_STREAM("use_pause: " << use_pause);
    if( use_robot )
    {
        moveit_node = new MoveItNode;
        Robot_GotoHome();
    }

    //geometry_msgs::Pose pose_observe;
    //pose_observe.orientation.x = 0.130513665483;
    //pose_observe.orientation.y = 0.991446495056;
    //pose_observe.orientation.z = 0.0;
    //pose_observe.orientation.w = 0.0;
    //pose_observe.position.x = 0.500;
    //pose_observe.position.y = 0.000;
    //pose_observe.position.z = 0.750;

    geometry_msgs::Pose pose_push_pre;
    pose_push_pre.orientation.x = -0.60879131828;
    pose_push_pre.orientation.y =  0.793330252171;
    pose_push_pre.orientation.z = -0.000118264808139;
    pose_push_pre.orientation.w = -0.000507681752219;
    pose_push_pre.position.x = 0.500;
    pose_push_pre.position.y = 0.000;
    pose_push_pre.position.z = 0.400;

    Eigen::Quaternionf q_finger_push_x( -0.000507681752219,
                                        -0.608791318280000,
                                         0.793330252171000,
                                        -0.000118264808139 );

    Eigen::Quaternionf q_finger_z180(0,0,0,1);

    // Get Parameters
    string fp_config, camera_name, object_name, dp_save;
    int id_push;
    double finger_length;
    nh.getParam(prefix_param + "camera_config",fp_config);
    nh.getParam(prefix_param + "camera_name",camera_name);
    nh.getParam(prefix_param + "object_name",object_name);
    nh.getParam(prefix_param + "save_dir",dp_save);
    nh.getParam(prefix_param + "push_point",id_push);
    nh.getParam(prefix_param + "finger_length",finger_length);

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
    string marker_type = "none";
    nh.getParam(prefix_param + "marker_type",marker_type);
    
    double marker_size;
    nh.getParam(prefix_param + "marker_size",marker_size);    
    marker_manager = new MarkerManager(nh,caminfo,marker_size);

    if( use_robot )
    {
        // Move to observed mode
        ROS_INFO("[Move] observation pose");
        //moveit_node->plan_and_execute(pose_observe);
        Robot_GotoObservation();
        ROS_INFO("[Move] observation pose - done");
    }

    // Get observation - BEGIN    
    ROS_INFO("[Camera] sleep 2 secs");
    ros::Duration(2).sleep();
    ROS_INFO("[Camera] sleep 2 secs - done");    
    ROS_INFO("[Camera] detect markers");
    cv::Mat image_0, depth_0, label_0;
    vector<Marker> markers_0, markers_push_0;
    Eigen::Matrix4f tf_cam2world_0;
    GetMarkers(nh,image_0,depth_0,label_0,caminfo,tf_cam2world_0,markers_0,
               &pub_begin_cloud, &pub_begin_marker,1,1,0,
               &markers_push_0, &pub_begin_push,0,1,0     );

    ROS_INFO("[Camera] detect markers - done");

    if( id_push==-1 )
    {
        string str_id = WaitUserInput("Please enter the pushing point id: ");
        stringstream ss;
        ss << str_id;
        ss >> id_push;        
    }

    // Find the target point
    bool found = false;
    for( int i=0; i<markers_push_0.size(); i++ )
    {
        if( markers_push_0[i].id==id_push )
        {
            pose_push_pre.position.x = markers_push_0[i].pose.position.x;
            pose_push_pre.position.y = markers_push_0[i].pose.position.y;
            pose_push_pre.position.z = markers_push_0[i].pose.position.z + finger_length + 0.10;
            found = true;
            break;
        }
    }
    if( found==false )
    {
        ROS_ERROR_STREAM("cannot find the target pushing point: " << id_push);
        Robot_GotoHome();
        return 0;
    }

    if( use_robot )
    {
        double push_x = id_push < 8 ? 0.12: -0.12; // 0.02 finger offset
        Eigen::Quaternionf q_finger;
        if( push_x >= 0 )
        {
            q_finger = q_finger_push_x;
        }
        else
        {
            q_finger = q_finger_z180 * q_finger_push_x;
        }
        pose_push_pre.orientation.x = q_finger.x();
        pose_push_pre.orientation.y = q_finger.y();
        pose_push_pre.orientation.z = q_finger.z();
        pose_push_pre.orientation.w = q_finger.w();
        
        moveit_node->plan_and_execute(pose_push_pre);
        WaitUserConfirm();
        ROS_INFO("[Move] be ready for push");
        // finger x offset:0.020, z buffer: 0.10
        moveit_node->plan_and_execute_via_waypoints(0, 0,-0.10, 20);
        ROS_INFO("[Move] be ready for push - done");
    
        //ros::Duration(2).sleep();
        WaitUserConfirm();
    
        ROS_INFO("[Move] Push!!!");    
        ros::Time time_push_beg = ros::Time::now();
        moveit_node->plan_and_execute_via_waypoints(push_x, 0, 0, 20);
        ros::Time time_push_end = ros::Time::now();
        double time_push = (time_push_end-time_push_beg).toSec();
        ROS_INFO("[Move]Push - done");
    
        //ros::Duration(2).sleep();
        WaitUserConfirm();
    
        ROS_INFO("[Move] observation pose");
        moveit_node->plan_and_execute_via_waypoints(0, 0, 0.10, 20);
        ros::Duration(1).sleep();
        //moveit_node->plan_and_execute_via_waypoints(pose_observe);
        Robot_GotoObservation();
        ROS_INFO("[Move] observation pose - done");

        // Get observation - FINAL
        ROS_INFO("[Camera] sleep 2 secs");
        ros::Duration(2).sleep();
        ROS_INFO("[Camera] sleep 2 secs - done");
        ROS_INFO("[Camera] detect markers");
        cv::Mat image_1, depth_1, label_1;
        vector<Marker> markers_1;
        Eigen::Matrix4f tf_cam2world_1;
        GetMarkers(nh,image_1,depth_1,label_1,caminfo,tf_cam2world_1,markers_1,
                   &pub_final_cloud, &pub_final_marker,0,0,1);
        ROS_INFO("[Camera] detect markers - done");

        vector<Marker> markers_push_1;
        geometry_msgs::Pose tf_pose;
        Tracking( caminfo, 
                  markers_0, markers_1, 
                  tf_cam2world_0, tf_cam2world_1,
                  markers_push_0, markers_push_1,
                  tf_pose,
                  &pub_final_push );
    
        ROS_INFO("[Save] saving data ...");
        save(dp_save+"/"+object_name, image_0, depth_0, label_0, markers_push_0, 
                                      tf_pose,
                                      image_1, depth_1, label_1, markers_push_1, id_push, push_x - 0.02, time_push);
        ROS_INFO("[Save] saving data ... done");

        ROS_INFO("[Move] goto home");
        Robot_GotoHome();
        ROS_INFO("[Move] goto Home - done");
    }

    if( marker_manager ) delete marker_manager;

    return 0;
}
