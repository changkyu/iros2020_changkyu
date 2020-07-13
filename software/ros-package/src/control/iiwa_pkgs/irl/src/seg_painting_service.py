#!/usr/bin/env python
'''
File: seg_painting_service.py

Authors: Aravind Sivaramakrishnan

Description: Calls the segmentation/lccp_2Dseg service, and for each of the returned objects, computes
a task_planning/ObjectPose message for the application to use.

Comments/TODO:
- Need a fix for this namespace issue!
'''
import rospy
import tf
import numpy as np
from task_planning.msg import ObjectPose
from task_planning.srv import SegmentationCall, SegmentationCallResponse
from rl_msgs.srv import seg_scene_srv
from rl_msgs.msg import SegmentationObject, SegmentationFace, SegmentationScene
from geometry_msgs.msg import PoseStamped
import rospkg

np.set_printoptions(suppress=True)

rospack = rospkg.RosPack()
cam_info_path = rospack.get_path('camera_calib')

canvas_quat = np.array([-0.27059805,  0.65328148,  0.65328148,  0.27059805])

class Surface:
	def __init__(self,face):
		self.camera_RT = np.loadtxt(cam_info_path+"/input/cam_info.txt")

		center_ = np.array([face.center.x,face.center.y,face.center.z,1.0])
		self.center = np.dot(self.camera_RT,center_)[:-1]

		normal_ = np.array([face.normal.x,face.normal.y,face.normal.z])
		self.normal = np.dot(self.camera_RT[:3,:3],normal_)

		self.axes = np.zeros((3,3))
		for x in range(3):
			axis_ = [face.vecs_axis[x].x, face.vecs_axis[x].y, face.vecs_axis[x].z]
			self.axes[x] = np.dot(self.camera_RT[:3,:3], axis_)
			self.axes[x] = self.axes[x]/np.linalg.norm(self.axes[x])

		self.sizes = face.sizes


class ObjectMessageServer:
	def __init__(self):
		rospy.init_node("object_message_server",disable_signals=True)
		self.service = rospy.Service("/iiwa/object_message",SegmentationCall,self.call_service)

		self.threshold = 20000 
		self.table_height = -0.25

		rospy.wait_for_service("segmentation/lccp_2Dseg")
		self.segmentation_service = rospy.ServiceProxy("segmentation/lccp_2Dseg",seg_scene_srv)
		
		self.camera_RT = np.loadtxt(cam_info_path+"/input/cam_info.txt")

		self.surfaces = []

	def find_best_grasp(self,idx,tnorm):
		best_match_idx = 0
		best_match = -1

		for j in range(len(self.surfaces[idx])):
			dot_pdt = np.abs(np.dot(self.surfaces[idx][j].normal,tnorm))
			if dot_pdt > best_match:
				best_match = dot_pdt
				best_match_idx = j

		return best_match_idx

	def calculate_rotation(self,axes,offset=45*np.pi/180.0):
		world_frame = np.identity(4) # Identity matrix for rotation, no translation
		current_frame = np.identity(4) # no translation

		for i in range(3):
			current_frame[i,:3] = axes[i]

		tf_matrix = np.dot(current_frame,world_frame)

		rotation_quaternion_ = tf.transformations.quaternion_from_matrix(tf_matrix)

		offset_quaternion = tf.transformations.quaternion_about_axis(offset,[0,0,1])

		# rotation_quaternion = tf.transformations.quaternion_multiply(rotation_quaternion_,offset_quaternion)
		rotation_quaternion = tf.transformations.quaternion_multiply(offset_quaternion,rotation_quaternion_)

		return tf.transformations.unit_vector(rotation_quaternion)

	def generate_object_message(self,i,j,tnorm):
		surface_of_interest = self.surfaces[i][j]
		axes = surface_of_interest.axes

		axes[2] = tnorm
		axes[0] = np.cross(axes[1],tnorm)

		if np.dot(axes[2],tnorm) < 0:
			# print "Ternary axis and reference normal not in the same direction!"
			axes[2] = -axes[2]
		if np.dot(np.cross(axes[2],axes[0]),axes[1]) < 0:
			# print "Right hand rule violated!"
			axes[0] = -axes[0]

		quat = self.calculate_rotation(axes)
		center = surface_of_interest.center

		message = ObjectPose()
		message.pose.orientation.x = quat[0]
		message.pose.orientation.y = quat[1]
		message.pose.orientation.z = quat[2]
		message.pose.orientation.w = quat[3]

		message.pose.position.x = center[0]
		message.pose.position.y = center[1]
		message.pose.position.z = center[2]

		print "Quaternion of brush: " + repr(quat)

		message.label = "brush"

		return message

	def find_brush(self):
		supervoxel_size_ratios = np.zeros((len(self.surfaces),))
		for i in range(len(self.surfaces)):
			for j in range(len(self.surfaces[i])):
				if self.surfaces[i][j].sizes[0] < 0.2:
					# Just to ensure that canvas is not considered...
					supervoxel_size_ratios[i] += self.surfaces[i][j].sizes[0]/(self.surfaces[i][j].sizes[1]*len(self.surfaces[i]))
		print supervoxel_size_ratios
		self.brush_idx = np.argmax(supervoxel_size_ratios)
		print "Found brush  in supervoxel " + str(self.brush_idx) + " and location:" + repr(self.surfaces[self.brush_idx][0].center)	

	def find_canvas(self,brush_offset=0.2,tnorm=[1,0,0]):
		voxel_sizes = []
		for i in range(len(self.surfaces)):
			current_max = 0
			for j in range(len(self.surfaces[i])):
				current_max = max(max(self.surfaces[i][j].sizes),current_max)
			voxel_sizes.append(current_max)
		self.canvas_idx = np.argmax(voxel_sizes)
		print "Found canvas in supervoxel " + str(self.canvas_idx) + " and location:" + repr(self.surfaces[self.canvas_idx][0].center)

		# Find a point that is brush_offset away from the center, along the normal.
		axes = self.surfaces[self.canvas_idx][0].axes

		print "Original axes: "
		print axes

		axes[2] = tnorm
		# axes[0] = np.cross(axes[1],tnorm)

		print "Projected axes: "
		print axes

		height_offset = 0.4
		point_offset = 0.2

		# First, find a better point to reach for.
		pt = self.surfaces[self.canvas_idx][0].center 
		pt += point_offset*axes[0]
		pt -= height_offset*axes[1]
		pt -= brush_offset*axes[2]

		# pt += height_offset*axes[:,0]
		# pt += brush_offset*axes[:,1]
		# pt += point_offset*axes[:,2]

		pose = ObjectPose()
		print "Actual point of approach: " + repr(pt)
		pose.pose.position.x = pt[0]
		pose.pose.position.y = pt[1]
		pose.pose.position.z = pt[2]

		print "Quaternion of approach: " + repr(canvas_quat)
		pose.pose.orientation.x = canvas_quat[0]
		pose.pose.orientation.y = canvas_quat[1]
		pose.pose.orientation.z = canvas_quat[2]
		pose.pose.orientation.w = canvas_quat[3]

		pose.label = "canvas"

		return pose


	def call_service(self,req):
		# Make the service call to the segmentation service
		try:
			resp = self.segmentation_service(use_camera=True)
			for i in range(len(resp.segscene.objects)):
				center_ = [resp.segscene.objects[i].center.x,resp.segscene.objects[i].center.y,resp.segscene.objects[i].center.z,1]
				center  = np.dot(self.camera_RT,center_)
				if len(resp.segscene.objects[i].cloud.data) > self.threshold and center[2] > self.table_height:
					supervoxel = []
					for j in range(len(resp.segscene.objects[i].faces)):
						surface = Surface(resp.segscene.objects[i].faces[j])
						supervoxel.append(surface)
					self.surfaces.append(supervoxel)
		except rospy.ServiceException, e:
			print "Service call to segmentation service failed: %s"%e

		object_messages = []

		# tnorm = np.array([req.normal.x, req.normal.y, req.normal.z])
		tnorm = np.array([0,0,-1])

		if np.linalg.norm(tnorm) == 0 or tnorm is None:
			tnorm = [0,0,-1]

		print "Reference vector being used: " + str(tnorm)

		self.find_brush()
		gn = self.find_best_grasp(self.brush_idx,tnorm)
		object_messages.append(self.generate_object_message(self.brush_idx,gn,tnorm))

		canvas_pose = self.find_canvas()
		object_messages.append(canvas_pose)

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
    