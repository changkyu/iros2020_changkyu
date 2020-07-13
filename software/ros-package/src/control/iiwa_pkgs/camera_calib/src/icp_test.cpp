#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"

#include <Eigen/Core>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

using namespace pcl;

tf::TransformListener *tf_l;

// boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
// {
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// 	viewer->setBackgroundColor(0,0,0);
// 	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
// 	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
// 	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"sample cloud");
// 	viewer->initCameraParameters();
// 	return(viewer);
// }

int main (int argc, char** argv)
{
	std::string from_frame = "world";
	std::string to_frame = "iiwa_joint_6";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	std::string file_target = "test.pcd";

	ros::init(argc,argv,"icp_test");
	ros::NodeHandle node_handle;
	tf_l = new tf::TransformListener;
	tf::StampedTransform camera_tf;

	sensor_msgs::PointCloud2::ConstPtr msg3 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/pointcloud",node_handle,ros::Duration(5.0));

	PointCloudRGB::Ptr cloud_xyzrgb(new PointCloudRGB);
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg3,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*cloud_xyzrgb);

	pcl::copyPointCloud(*cloud_xyzrgb, *cloud_in);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_target, *cloud_out) == -1)
	{
		return -1;
	}

	try 
	{
		tf_l->waitforTransform(from_frame, to_frame, ros::Time(0), ros::Duration(5));
		tf_l->lookupTransform(from_frame, to_frame, ros::Time(0), camera_tf);
		
	}
}