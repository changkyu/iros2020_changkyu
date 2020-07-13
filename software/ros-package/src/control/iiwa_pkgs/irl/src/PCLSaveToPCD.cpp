#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <rl_msgs/seg_scene_srv.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pcl_test");
	ros::NodeHandle nh;

	std::string dname = "/home/cm1074/aravind/Notebooks/data00/";
	std::string fname = argv[1];

	pcl::PCLPointCloud2 cloud2;
	pcl::PointCloud<PointT>::Ptr cloud_xyzrgb(new pcl::PointCloud<PointT>);

	Eigen::Matrix4d transformer(4,4);
	transformer << -4.331774739200174029e-02, -6.074583788962458764e-01, 7.983562514792436060e-01, 2.459879231277625755e-01,
					-1.003889096390860436e+00, 8.931393968872393049e-03, -4.226192804062041050e-03, 9.173754795683436783e-03,
					3.311304989276537582e-03, -7.995874138969585321e-01, -6.013225595589501982e-01, 8.045004059125550333e-02,
					0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00;

	sensor_msgs::PointCloud2::ConstPtr message = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/pointcloud",nh,ros::Duration(5.0));
	pcl_conversions::toPCL(*message,cloud2);
	pcl::fromPCLPointCloud2(cloud2,*cloud_xyzrgb);
	pcl::transformPointCloud(*cloud_xyzrgb,*cloud_xyzrgb,transformer);

	ROS_INFO_STREAM("Writing to: " << dname+fname+".pcd");

	pcl::io::savePCDFileASCII(dname+fname+".pcd",*cloud_xyzrgb);

	return 0;
}