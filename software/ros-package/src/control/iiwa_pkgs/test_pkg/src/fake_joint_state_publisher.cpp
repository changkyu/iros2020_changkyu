#include "ros/ros.h"
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "fake_joint_state_publisher");
	ros::NodeHandle* rosnode = new ros::NodeHandle();

	ros::Publisher pub;
	pub = rosnode->advertise<sensor_msgs::JointState>(
    "/iiwa/joint_states", 1, true);
	ros::Rate loop_rate(100);
    while(ros::ok()){
    	sensor_msgs::JointState msg;
		msg.name.push_back("iiwa_joint_ee");
		msg.position.push_back(0); 
		pub.publish(msg);
		ros::spinOnce();
  
      	loop_rate.sleep();
    }
    return 0;
}