#!/usr/bin/env python
'''
File: calculate_extrinsic_params.py
Authors: Aravind Sivaramakrishnan
Description: 
Comments/TODO:
- Currently moving to each point manually using the test script. Should probably find a way to automate this.
'''
import numpy as np
import ast
import rospkg
import tf

np.set_printoptions(suppress=True)

rospack = rospkg.RosPack()
package_path = rospack.get_path("camera_calib")

offset_angle = 0 #45
depth_scale = 1.0

# patch_offset = np.array([0.063,0.005,0.086,1.0])
# patch_offset = np.array([0.063,0.005,0.086,1.])
patch_offset = np.array([0,0,0,1.])
# patch_offset = np.array([0,0.06,-0.042,1.])
# patch_offset = np.array([0,0.0634,-0.044,1.])
#patch_offset = np.array([0,0.064,-0.04,1.])

offset_quat = tf.transformations.quaternion_about_axis(offset_angle*np.pi/180.0,[0,0,1])
patch_local = np.dot(tf.transformations.quaternion_matrix(offset_quat),patch_offset)
# raw_input(patch_local)
def transform_train(point):
	'''Given location of the end effector in the world frame, find the location of the patch
	in the world frame.'''
	tf_matrix = tf.transformations.quaternion_matrix(point[3:])
	tf_matrix[:3,3] = point[:3]
	return np.dot(tf_matrix,patch_local)[:3]


def point_error(cam_point,ee_point,cam_matrix):
	'''Given location of the patch in the world frame (as found by the camera), find the location of
	the patch in the local frame (patch_offset).'''
	tf_matrix = tf.transformations.quaternion_matrix(ee_point[3:])
	tf_matrix[:3,3] = ee_point[:3]
	# raw_input("ee_point "+str(ee_point))
	patch_world = np.dot(tf_matrix,patch_local)
	# raw_input(patch_world)
	patch_camera = np.dot(cam_matrix,patch_world)

	print "Calculated pose of the patch (cam): " + str(patch_camera[:3])
	print "Actual pose of the patch (cam)    : " + str(cam_point[:3])
	print "Abs Diffs: " + str(abs(patch_camera[:3] - cam_point[:3]))

	error = np.linalg.norm(patch_camera-cam_point)
	print "Error is: " + str(error)
	return error

def test_calculations(cam_mat_inv,start):
	error = 0.0
	for i in range(start,start+4):
		with open(package_path+"/input/cam_pose_"+str(i)+".txt") as f:
			temp_cam = []
			for l in f:
				temp_cam.append(ast.literal_eval(l)*depth_scale)
			temp_cam.append(1.0)
		with open(package_path+"/input/ee_pose_"+str(i)+".txt") as f:
			temp_ee = []
			for l in f:
				temp_ee.append(ast.literal_eval(l))
		error += point_error(temp_cam,temp_ee,cam_mat_inv)
	print "Average error: " + str(error/4)


train_method = "lsq"
# train_method = "inv"
train_split = {"lsq":12, "inv":4}

A = np.zeros((3*train_split[train_method],12))
ee_coordinates = np.zeros((3*train_split[train_method],))

for i in range(train_split[train_method]):
	with open(package_path+"/input/cam_pose_"+str(i)+".txt") as f:
		temp = []
		for l in f:
			temp.append(ast.literal_eval(l)*depth_scale)
	A[(i*3),:3]     = temp[:3]
	A[(i*3),3]	    = 1.0
	A[(i*3)+1,4:7]  = temp[:3]
	A[(i*3)+1,7]    = 1.0
	A[(i*3)+2,8:11] = temp[:3]
	A[(i*3)+2,11]   = 1.0

	print '\n\n\n', A, '\n\n\n'

	with open(package_path+"/input/ee_pose_"+str(i)+".txt") as f:
		temp = []
		for l in f:
			temp.append(ast.literal_eval(l))
		ee_coordinates[i*3:(i+1)*3] = transform_train(temp)

cam_extrinsic_matrix = np.identity(4)

if train_method == "lsq":
	cam_extrinsic_matrix[:3,:] = np.linalg.lstsq(A,ee_coordinates)[0].reshape(3,4)
else:
	cam_extrinsic_matrix[:3,:] = np.dot(np.linalg.inv(A),ee_coordinates).reshape(3,4)

print cam_extrinsic_matrix

cam_mat_inv = np.identity(4)
cam_mat_inv[:3,:3] = np.linalg.inv(cam_extrinsic_matrix[:3,:3])
cam_mat_inv[:3,3]   = np.dot(-cam_mat_inv[:3,:3],cam_extrinsic_matrix[:3,3])

print cam_mat_inv

final_quaternion = tf.transformations.quaternion_from_matrix(cam_extrinsic_matrix)

np.savetxt(package_path+"/input/cam_info.txt",cam_extrinsic_matrix)

print 'pose(xyz-xyzw): ', cam_extrinsic_matrix[:3,3][0],",",cam_extrinsic_matrix[:3,3][1],",",cam_extrinsic_matrix[:3,3][2],",", final_quaternion[3],",", final_quaternion[0],",", final_quaternion[1],",", final_quaternion[2]
# print 'translation: ', cam_extrinsic_matrix[:3,3]
# print cam_extrinsic_matrix[:3,3][0],cam_extrnisic_matrix[:3,3][1],cam_extrinsic_matrix[:3,3][2], final_quaternion.w, final_quaternion.x, final_quaternion.y, final_quaternion.z

test_calculations(cam_mat_inv,train_split[train_method])
print "TRAIN METHOD: ",train_split[train_method]
# test_calculations(cam_mat_inv,train_split[train_method])