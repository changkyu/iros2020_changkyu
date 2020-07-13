#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <limits>
#include "TaskPlanner.hpp"

using namespace std;
using namespace pcl;

PointXYZRGB point;
string ROS_TOPIC_POINT;
pcl::visualization::PCLVisualizer viewer("Segmentation Result");
PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloudWorld(new PointCloud<PointXYZRGB>);
string outdir;
Eigen::Matrix4f camPose;
std::vector<geometry_msgs::PoseStamped> calib_poses_vector;
bool running = true;
bool go_to_next = false;
bool compute = false;
bool refresh = false;
bool detection_ready = false;
int idx=0;



void Callback_point(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::cout<<"Callback_point"<<std::endl;
    static bool busy=false;
    if( busy ) return;
    busy=true;


    fromROSMsg<PointXYZRGB>(*msg, *cloud);
    
    // realsense camera package does not stream in meters
    for(int ii = 0; ii< cloud->size(); ii++) {
        cloud->points[ii].x = cloud->points[ii].x*0.1;
        cloud->points[ii].y = cloud->points[ii].y*0.1;
        cloud->points[ii].z = cloud->points[ii].z*0.1;
        
    }
    vector<int> index;
    removeNaNFromPointCloud(*cloud,*cloud,index);

    pcl::transformPointCloud(*cloud, *cloudWorld, camPose);
    pcl::io::savePLYFile("/home/cm1074/Desktop/abc.ply", *cloud);

    PointCloud<PointXYZRGB>::Ptr cloud_points_pruned(new PointCloud<PointXYZRGB>);
    std::vector<PointXYZRGB*> color_point_vector;
    std::vector<PointXYZRGB*> filtered_color_point_vector;
    PointXYZRGB *color_point;
    for(int cc = 0; cc < cloudWorld->size(); cc++){
        uint32_t rgb = *reinterpret_cast<int*>(&cloudWorld->points[cc].rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;

        //ROS_WARN_STREAM("cloud:"<<int(r)<<","<<int(g)<<","<<int(b));
        //ROS_WARN_STREAM("cloud:"<<cloud->points[cc].r);
        if(r > 100 && r < 130 && g < 70 && g > 50 && b < 70 && b > 50){
            //color_point = new PointXYZRGB();
            color_point = &cloudWorld->points[cc];
            color_point_vector.push_back(color_point);
            cloud_points_pruned->points.push_back(*color_point);
        }
    }
    pcl::io::savePLYFile("/home/cm1074/Desktop/abc_pruned.ply", *cloud_points_pruned);

    double color_x = 0;
    double color_y = 0;
    double color_z = 0;
    ROS_WARN_STREAM("red point size:"<<color_point_vector.size());
    if(color_point_vector.size()>0){ 
        for(auto c_point: color_point_vector){
            color_x += c_point->x;
            color_y += c_point->y;
            color_z += c_point->z;

        }
        color_x = color_x / color_point_vector.size();
        color_y = color_y / color_point_vector.size();
        color_z = color_z / color_point_vector.size();
        int nearest_index = 0;
        double nearest_dist = std::numeric_limits<double>::max();
        for(int cc = 0; cc <  color_point_vector.size();cc++){
            double dist = pow(color_point_vector[cc]->x - color_x, 2) + pow(color_point_vector[cc]->y - color_y, 2) + pow(color_point_vector[cc]->z - color_z, 2);
            if(dist < nearest_dist){
                nearest_index = cc;
                nearest_dist = dist;
            }

        }
        point = *color_point_vector[nearest_index];
        std::cout<<"point:"<<point.x<<","<<point.y<<","<<point.z<<std::endl;
        // if(cloud->size() > 0)
        // {        
        //     viewer.updatePointCloud(cloud);

        //     viewer.removeAllShapes();
        //     PointXYZRGB pt_x1 ,pt_x2, pt_y1, pt_y2, pt_z1, pt_z2;
        //     pt_x1 = point; pt_x2 = point;
        //     pt_y1 = point; pt_y2 = point;
        //     pt_z1 = point; pt_z2 = point;
        //     pt_x1.x = point.x - 0.01; pt_x2.x = point.x + 0.01;
        //     pt_y1.y = point.y - 0.01; pt_y2.y = point.y + 0.01;
        //     pt_z1.z = point.z - 0.01; pt_z2.z = point.z + 0.01;
        //     viewer.addLine(pt_x1, pt_x2, 0,0,1, "pt_x" );
        //     viewer.addLine(pt_y1, pt_y2, 0,0,1, "pt_y" );
        //     viewer.addLine(pt_z1, pt_z2, 0,0,1, "pt_z" );

        //     viewer.spinOnce(10);        
        // }
    }
    busy=false;
    
}

static
void Callback_viewer(const visualization::KeyboardEvent &event, void* viewer_void)
{
    visualization::PCLVisualizer *viewer
     = static_cast<visualization::PCLVisualizer *> (viewer_void);
    if (event.keyDown())
    {
        if( event.getKeySym()=="Escape" )
        {
            running = false;            
        }
        else if( event.getKeySym()=="space")
        {
            go_to_next = true;
        }
        else if( event.getKeySym()=="Return" )
        {
            compute = true;
        }        
        else if( event.getKeySym()=="F5" )
        {            
            refresh = true;
        }
    }
}

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    if (event.getPointIndex () == -1)
    {
      return;
    }

    point = cloud->points[event.getPointIndex()];
    
    viewer.removeAllShapes();
    PointXYZRGB pt_x1 ,pt_x2, pt_y1, pt_y2, pt_z1, pt_z2;
    pt_x1 = point; pt_x2 = point;
    pt_y1 = point; pt_y2 = point;
    pt_z1 = point; pt_z2 = point;
    pt_x1.x = point.x - 0.01; pt_x2.x = point.x + 0.01;
    pt_y1.y = point.y - 0.01; pt_y2.y = point.y + 0.01;
    pt_z1.z = point.z - 0.01; pt_z2.z = point.z + 0.01;

    viewer.addLine(pt_x1, pt_x2, 0,0,1, "pt_x" );
    viewer.addLine(pt_y1, pt_y2, 0,0,1, "pt_y" );
    viewer.addLine(pt_z1, pt_z2, 0,0,1, "pt_z" );

    ROS_INFO_STREAM("select point: " << point);
}

void toTransformationMatrix(Eigen::Matrix4f& camPose, std::vector<double> camPose7D){
    camPose(0,3) = camPose7D[0];
    camPose(1,3) = camPose7D[1];
    camPose(2,3) = camPose7D[2];
    camPose(3,3) = 1;

    Eigen::Quaternionf q;
    q.w() = camPose7D[3];
    q.x() = camPose7D[4];
    q.y() = camPose7D[5];
    q.z() = camPose7D[6];
    Eigen::Matrix3f rotMat;
    rotMat = q.toRotationMatrix();

    for(int ii = 0;ii < 3; ii++)
        for(int jj=0; jj < 3; jj++){
            camPose(ii,jj) = rotMat(ii,jj);
        }
}

// overload opertor << for std::vector<double> 
std::ostream& operator<<(std::ostream &out, std::vector<double> &double_vec)
{
    for (int count=0; count < double_vec.size(); ++count)
    {
        out << double_vec[count] << " ";
    }
    out << std::endl;

    return out;
}

void loadCalibPoses(int binId)
{
    ros::NodeHandle nh;
    std::string resources_path = ros::package::getPath("chimp_resources");
    system(("rosparam load " +resources_path + "/config/lower_grid_calib.yaml").c_str());
// read bin pose
    for(int i = 0; i < 24; i ++){
        std::vector<double> calib_poses;
        geometry_msgs::PoseStamped pose;
        nh.getParam("bin_"+std::to_string(binId)+"/id_"+std::to_string(i), calib_poses);  
        pose.pose.position.x = calib_poses[0];
        pose.pose.position.y = calib_poses[1];
        pose.pose.position.z = calib_poses[2];
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 1;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 0;

        calib_poses_vector.push_back(pose);
    }
    
    //for(int j = 0; j < 24; j ++){
       // std::cout<<calib_poses_vector[j];
    //}
}

static
void ParseParam(ros::NodeHandle nh)
{
    nh.param<string>("/camera_calib/subtopic/pointcloud",ROS_TOPIC_POINT,"/camera/depth_registered/points");
    nh.param<string>("/camera_calib/outdir",outdir,getenv("HOME"));
}

int main( int argc, char** argv )
{
    ros::init(argc,argv,"camera_calib");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
//     ros::MultiThreadedSpinner spinner(4); // Use 4 threads
// spinner.spin(); // spin() will not return until the node has been shutdown
    std::cout << "starting the service " << std::endl;
    ros::NodeHandle nh;
    ParseParam(nh);
    bool first_time = true;
    ros::Subscriber sub_point = nh.subscribe(ROS_TOPIC_POINT,10,Callback_point);
    
    viewer.setPosition(0,0);
    viewer.setSize(600,480);
    viewer.setCameraPosition(0,0,-1,0,0,1,0,-1,0);
    viewer.addPointCloud(cloud);
    //viewer.registerKeyboardCallback(Callback_viewer, (void*)&viewer);
   // viewer.registerPointPickingCallback (pp_callback, (void*)&viewer); 
    int binId = 2;
    char cam_pose_topic[100];
    sprintf(cam_pose_topic,"/bins/bin_%d/camera_pose", binId);
    std::string resources_path = ros::package::getPath("chimp_resources");
    system(("rosparam load " +resources_path + "/config/bin_config.yaml").c_str());
    std::vector<double> camPose7D;
    nh.getParam(cam_pose_topic, camPose7D);
    camPose = Eigen::Matrix4f::Zero(4,4);
    toTransformationMatrix(camPose, camPose7D);
    std::cout << "Camera Pose: " << std::endl << camPose << std::endl;


    TaskPlanner task_planner;
    MoveItNode *moveit_node = task_planner.get_planning_node();
    binId = 2;
    loadCalibPoses(binId);
    int calib_pose_id = 0;
    //moveit_node->move_grid();
    ROS_WARN("before running");
    while( ros::ok() && running )
    {   //ROS_WARN("running");
        //ros::spinOnce();
        viewer.spinOnce(3);
        if(first_time){
            moveit_node->plan_and_execute(calib_poses_vector[calib_pose_id]);
            calib_pose_id ++;
            first_time = false;
        }
        //if( go_to_next )
        //{
            stringstream ss;
            ss << outdir << "/ee_pose_" << idx << ".txt";
            ROS_INFO_STREAM("write to " << ss.str());

            ofstream f_ee(ss.str());
            geometry_msgs::PoseStamped p = task_planner.get_current_pose();
            f_ee << p.pose.position.x << endl;
            f_ee << p.pose.position.y << endl;
            f_ee << p.pose.position.z << endl;
            f_ee << p.pose.orientation.x << endl;
            f_ee << p.pose.orientation.y << endl;
            f_ee << p.pose.orientation.z << endl;
            f_ee << p.pose.orientation.w << endl;
            f_ee.close();
            ROS_INFO_STREAM("done");

            stringstream ss_cam;
            ss_cam << outdir << "/cam_pose_" << idx << ".txt";
            ROS_INFO_STREAM("write to " << ss_cam.str());

            ofstream f_cam(ss_cam.str());
            f_cam << point.x << endl;
            f_cam << point.y << endl;
            f_cam << point.z << endl;
            f_cam.close();
            ROS_INFO_STREAM("done");
             detection_ready = false;
            idx++;
            viewer.updatePointCloud(cloud);

            viewer.removeAllShapes();
            PointXYZRGB pt_x1 ,pt_x2, pt_y1, pt_y2, pt_z1, pt_z2;
            pt_x1 = point; pt_x2 = point;
            pt_y1 = point; pt_y2 = point;
            pt_z1 = point; pt_z2 = point;
            pt_x1.x = point.x - 0.01; pt_x2.x = point.x + 0.01;
            pt_y1.y = point.y - 0.01; pt_y2.y = point.y + 0.01;
            pt_z1.z = point.z - 0.01; pt_z2.z = point.z + 0.01;
            viewer.addLine(pt_x1, pt_x2, 0,0,1, "pt_x" );
            viewer.addLine(pt_y1, pt_y2, 0,0,1, "pt_y" );
            viewer.addLine(pt_z1, pt_z2, 0,0,1, "pt_z" );

            viewer.spinOnce(10);        
            go_to_next = false;
            moveit_node->plan_and_execute(calib_poses_vector[calib_pose_id]);
            calib_pose_id ++;
            ros::Duration(7).sleep();
            detection_ready = true;
        //}
        if( refresh )
        {
            PointXYZRGB point_new;
            float dist_min = INFINITY;            
            for( size_t p=0; p<cloud->size(); p++ )
            {
                if( cloud->points[p].z < 0.01 ) continue;

                int dist_r = point.r - cloud->points[p].r;
                int dist_g = point.g - cloud->points[p].g;
                int dist_b = point.b - cloud->points[p].b;
                float dist = sqrt(dist_r*dist_r+dist_g*dist_g+dist_b*dist_b);
                if( dist < dist_min)
                {
                    dist_min = dist;
                    point_new = cloud->points[p];
                }
            }
            point = point_new;
            ROS_INFO_STREAM("point_new: " << point);

            viewer.removeAllShapes();
            PointXYZRGB pt_x1 ,pt_x2, pt_y1, pt_y2, pt_z1, pt_z2;
            pt_x1 = point; pt_x2 = point;
            pt_y1 = point; pt_y2 = point;
            pt_z1 = point; pt_z2 = point;
            pt_x1.x = point.x - 0.01; pt_x2.x = point.x + 0.01;
            pt_y1.y = point.y - 0.01; pt_y2.y = point.y + 0.01;
            pt_z1.z = point.z - 0.01; pt_z2.z = point.z + 0.01;

            viewer.addLine(pt_x1, pt_x2, 0,0,1, "pt_x" );
            viewer.addLine(pt_y1, pt_y2, 0,0,1, "pt_y" );
            viewer.addLine(pt_z1, pt_z2, 0,0,1, "pt_z" );

            refresh = false;
        }
    }
    ros::shutdown();

    return 0;
}
