#!/usr/bin/env python
'''
File: seg_stacking_service.py

Authors: Aravind Sivaramakrishnan

Description: Calls the segmentation/lccp_2Dseg service, and for each of the returned objects, computes
a task_planning/ObjectPose message for the application to use.

Comments/TODO:
- Need a fix for this namespace issue!
- Update the table height
'''
import rospy
import tf
import string
import numpy as np
from std_msgs.msg import Float32
from task_planning.msg import ObjectPose
from task_planning.srv import SegmentationCall, SegmentationCallResponse
from rl_msgs.srv import seg_scene_srv
from rl_msgs.msg import SegmentationObject, SegmentationFace, SegmentationScene
from geometry_msgs.msg import PoseStamped
import rospkg
import sys
np.set_printoptions(suppress=True)

rospack = rospkg.RosPack()
cam_info_path = rospack.get_path('camera_calib')

class ObjectMessageServer:
	def __init__(self):

		node_name = ""
		output_service_name = ""
		input_service_name = ""
		calibration_data = ""

		if sys.argv[1] == "1":
			node_name = "object_message_server_1"
			output_service_name = "/iiwa/object_message"
			input_service_name = "segmentation_1/lccp_2dseg"
			calibration_data = "/input/camera_1_calibration_data/cam_info.txt"
			self.workspace = [(0.39, -0.34, -0.25), (0.1,0.1,0.2)]
		elif sys.argv[1] == "2":
			node_name = "object_message_server_2"
			output_service_name = "/iiwa/object_message_2"
			input_service_name = "segmentation_2/lccp_2dseg"
			calibration_data = "/input/camera_2_calibration_data/cam_info.txt"
			self.workspace = [(0.39, 0.34, -0.25), (0.1,0.1,0.2)]
		else:
			exit(0)
 



		rospy.init_node(node_name)
		print "Initializing the segmentation service object_message_server"
		self.service = rospy.Service(output_service_name,SegmentationCall,self.call_service)

		self.threshold = 20000 
		self.table_height = -0.25	

		rospy.wait_for_service(input_service_name)
		self.segmentation_service = rospy.ServiceProxy(input_service_name,seg_scene_srv)
		print "Published the service..."

		self.camera_RT = np.loadtxt(cam_info_path+calibration_data)
		
		self.surfaces = []
		self.labels = list(string.ascii_uppercase)


	def find_best_grasp(self,idx,tnorm):
		best_match_idx = 0
		best_match = -1
		print "FIND BEST GRASP"
		for j in range(len(self.surfaces[idx])):
			print "----For surface ", j
			if not self.check_if_in_roi(idx,j):
				print "continuing"
				continue
			print "Doing the dot products for alignment"
			normal_ = [self.surfaces[idx][j].normal.x, self.surfaces[idx][j].normal.y, self.surfaces[idx][j].normal.z]
			normal = np.dot(self.camera_RT[:3,:3], normal_)
			dot_pdt = np.abs(np.dot(normal,tnorm))

			if dot_pdt > best_match:
				best_match = dot_pdt
				best_match_idx = j

		if best_match != -1 and abs(best_match) > 0.75:
			return best_match_idx
		else:
			return -1

	def calculate_rotation(self,axes,offset=45*np.pi/180.0):
		world_frame = np.identity(4) # Identity matrix for rotation, no translation
		current_frame = np.identity(4) # no translation

		for i in range(3):
			current_frame[i,:3] = axes[i]

		tf_matrix = np.dot(current_frame,world_frame)

		rotation_quaternion_ = tf.transformations.quaternion_from_matrix(tf_matrix)

		offset_quaternion = tf.transformations.quaternion_about_axis(offset,[0,0,1])

		rotation_quaternion = tf.transformations.quaternion_multiply(offset_quaternion,rotation_quaternion_)

		return tf.transformations.unit_vector(rotation_quaternion)

	def check_if_in_roi(self,i,j):
		surface_of_interest = self.surfaces[i][j]
		center_ = [surface_of_interest.center.x,surface_of_interest.center.y,surface_of_interest.center.z,1.0]
		center  = np.dot(self.camera_RT,center_)
		center = center[:-1]

		for i,coord in enumerate(center):
			if abs(coord - self.workspace[0][i]) > self.workspace[1][i] :
				print center, " was evaluated to be beyond ", self.workspace[1], "m of workspace center ", self.workspace[0]
				return False

		print center, " was within ", self.workspace[1], "m of workspace center ", self.workspace[0]
		return True


	def generate_object_message(self,i,j,tnorm):
		surface_of_interest = self.surfaces[i][j]
		axes = np.zeros((3,3))


		print "Dimensions "
		for x in range(len(surface_of_interest.vecs_axis)):
			print 2*surface_of_interest.sizes[x]


		for x in range(len(surface_of_interest.vecs_axis)):
			axis_ = [self.surfaces[i][j].vecs_axis[x].x, self.surfaces[i][j].vecs_axis[x].y, self.surfaces[i][j].vecs_axis[x].z]
			axes[x] = np.dot(self.camera_RT[:3,:3], axis_)
			axes[x] = axes[x]/np.linalg.norm(axes[x])


		axes[2] = tnorm
		axes[0] = np.cross(axes[1],tnorm)

		if np.dot(axes[2],tnorm) < 0:
			print "Tertiary axis not in the same direction!"
			# If the ternary axis of the surface is in the opposite direction of the reference normal
			axes[2] = -axes[2]
		# Check for right hand rule violation
		if np.dot(np.cross(axes[2],axes[0]),axes[1]) < 0:
			print "Right hand rule violated!"
			axes[0] = -axes[0]

		print "Primary, secondary and ternary axes: "
		print axes


		quat = self.calculate_rotation(axes)
		center_ = [surface_of_interest.center.x,surface_of_interest.center.y,surface_of_interest.center.z,1.0]
		center  = np.dot(self.camera_RT,center_) 
		print "p = np." + repr(center)
		print "q = np." + repr(quat)

		message = ObjectPose()
		message.pose.orientation.x = quat[0]
		message.pose.orientation.y = quat[1]
		message.pose.orientation.z = quat[2]
		message.pose.orientation.w = quat[3]

		message.pose.position.x = center[0]
		message.pose.position.y = center[1]
		message.pose.position.z = center[2]

		message.label = self.labels[i]

		message.sizes = []

		for size in surface_of_interest.sizes:
			size_ = Float32()
			size_.data = size
			message.sizes.append(size_)	

		return message

	def call_service(self,req):
		print "Call service function has been called..."
		# Make the service call to the segmentation service
		try:
			print "service_1 called"
			resp = self.segmentation_service(use_camera=True)
			print "Obtained a response..."
			for i in range(len(resp.segscene.objects)):
				print "Looping through the scene objects ",i
				center_ = [resp.segscene.objects[i].center.x,resp.segscene.objects[i].center.y,resp.segscene.objects[i].center.z,1]
				center  = np.dot(self.camera_RT,center_)
				print "object center: ", center
				if len(resp.segscene.objects[i].cloud.data) > self.threshold and center[2] > self.table_height:
					self.surfaces.append(resp.segscene.objects[i].faces)
		except rospy.ServiceException, e:
			print "Service call to segmentation service failed: %s"%e

		object_messages = []

		tnorm = np.array([req.normal.x, req.normal.y, req.normal.z])

		if np.linalg.norm(tnorm) == 0 or tnorm is None:
			tnorm = [0,0,-1]

		print "Reference vector being used: " + str(tnorm)

		for i in range(len(self.surfaces)):
			# Find the normal aligned along the surface normal to the table for each supervoxel
			print "\n\nEvaluating the object [",i,"]"
			gn = self.find_best_grasp(i,tnorm)
			# Generate a grasp message
			if gn != -1:
				print "Object ",i," has a best grasp within bounds..."
				object_messages.append(self.generate_object_message(i,gn,tnorm))

		self.surfaces = []

		print "---"

		return SegmentationCallResponse(object_messages=object_messages)

	def run(self):
		rospy.spin()

if __name__ == "__main__":
	try:
		object_message_server = ObjectMessageServer()
		object_message_server.run()
	except rospy.ROSInterruptException:
		pass
    