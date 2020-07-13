#!/usr/bin/env python
'''
File: tf_matrix_server.py

Authors: Aravind Sivaramakrishnan

Description: Sends the camera-to-world frame transformation matrix.

Comments/TODO:
'''
from camera_calib.srv import *
import rospy
import rospkg
import numpy as np

rospack = rospkg.RosPack()
package_path = rospack.get_path("camera_calib")

def get_tf_matrix(req):
	tf_matrix = np.loadtxt(package_path+"/input/cam_info.txt").reshape(16,)
	print tf_matrix.tolist()
	return TransformationMatrixResponse(tf_matrix.tolist())

def send_tf_matrix():
	rospy.init_node("tf_matrix_server")
	s = rospy.Service("tf_matrix",TransformationMatrix,get_tf_matrix)
	print "Ready to send the tf matrix."
	rospy.spin()

if __name__ == "__main__":
	send_tf_matrix()

	