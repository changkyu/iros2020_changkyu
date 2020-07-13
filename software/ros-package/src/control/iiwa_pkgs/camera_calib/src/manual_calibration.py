#!/usr/bin/env python
'''
File: manual_calibration.py

Authors: Aravind Sivaramakrishnan

Description: 

Comments/TODO:
- Must move a lot of things to the iiwa_ros namespace, looks like it might work out well.
- For some reason, rospy.wait_for_message() doesn't seem to work.
- Currently moving to each point manually using the test script. Should probably find a way to automate this.
'''
import rospy
import sys
import rospkg
from geometry_msgs.msg import PoseStamped, PointStamped

rospack = rospkg.RosPack()
package_path = rospack.get_path("camera_calib")

idx = -1

def ee_pose_cb(data):
	global ee_pose_sub
	with open(package_path+"/input/ee_pose_"+idx+".txt","wb") as f:
		f.write(str(data.pose.position.x)+"\n")
		f.write(str(data.pose.position.y)+"\n")
		f.write(str(data.pose.position.z)+"\n")
		f.write(str(data.pose.orientation.x)+"\n")
		f.write(str(data.pose.orientation.y)+"\n")
		f.write(str(data.pose.orientation.z)+"\n")
		f.write(str(data.pose.orientation.w)+"\n")
	print "EE co-ordinates registered!"
	ee_pose_sub.unregister()

def cam_pose_cb(data):
	global cam_pose_sub
	with open(package_path+"/input/cam_pose_"+idx+".txt","wb") as f:
		f.write(str(data.point.x)+"\n")
		f.write(str(data.point.y)+"\n")
		f.write(str(data.point.z)+"\n")
	print "Cam frame co-ordinates registered!"
	cam_pose_sub.unregister()

def register_point(arg):
	global ee_pose_sub, cam_pose_sub, idx
	idx = arg
	rospy.init_node("manual_calibration")
	ee_pose_sub = rospy.Subscriber("/iiwa/state/CartesianPose", PoseStamped, ee_pose_cb)
	cam_pose_sub = rospy.Subscriber("/clicked_point", PointStamped, cam_pose_cb)
	rate = rospy.Rate(0.1)

	while not rospy.is_shutdown():
		rate.sleep()

	rospy.spin()

if __name__ == "__main__":
	try:
		register_point(sys.argv[1])
	except rospy.ROSInterruptException:
		pass