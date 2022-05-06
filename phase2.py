#!/usr/bin/env python
"""
	Sample boiler-plate code for phase 1b
	Cyber Physical System Virtual Organization Challenge 2021 : SoilScope Lunar Lander ExoCam -- Earth Analog
	Team Name :
	Members :
"""

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Vector3Stamped, Quaternion
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, ActuatorControl, RCOut
from sensor_msgs.msg import Imu, Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header, Bool, Float64
from numpy import sqrt, pi, sin, cos, arctan2, array, linalg, tan, dot

from std_srvs.srv import Empty

g = 9.80665

class OffboardControl:
	""" Controller for PX4-UAV offboard mode """

	def __init__(self):
		rospy.init_node('OffboardControl', anonymous=True)

		# define your class variables here

		self.curr_pose = PoseStamped()                      # current pose of the drone
		self.des_pose = PoseStamped()                           # desired pose of the drone in position control mode
		self.is_ready_to_fly = False
		self.mode = "ASCEND"
		self.arm = False
		self.chute_detached = False
		self.att = AttitudeTarget()
		self.attach = False
		self.orientation = [0]*3
		self.attachFlag = True

		self.initializeVisionVariables()

		for i in reversed(range(1,4)):
			print "Launching node in {}...".format(i)
			rospy.sleep(1)

		# define ros services, subscribers and publishers here
		# arm or disarm the UAV
		self.armService = rospy.ServiceProxy('/uav/mavros/cmd/arming', CommandBool)
		# attach any two objects in Gazebo
		self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
		# detach any two attached objects
		self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
		# pause the Gazebo simulation if needed (could be used to debug the movement of the UAV)
		self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		# example call:
#		self.pause_physics_client.call()
		# could be used to reset the probe if needed
		self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

		# command your attitude target to the UAV
		self.att_setpoint_pub = rospy.Publisher('/uav/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
		# command a position setpoint to the UAV
		self.pos_setpoint_pub = rospy.Publisher('/uav/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		# publish the debug image for reference if needed
		self.debugImgPub = rospy.Publisher('/debug_cam',Image,queue_size=10)

		# get the current state of the UAV
		self.state_sub = rospy.Subscriber('/uav/mavros/state', State, callback=self.state_callback)
		# get the visual from the onboard camera
		self.img_sub = rospy.Subscriber('/uav_camera_down/image_raw',Image,self.img_cb)

		self.pose_sub = rospy.Subscriber('/uav/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)


		# call the state machine
		self.controller()

	def initializeVisionVariables(self):
		self.bridge = CvBridge()
		self.debug = False
		self.imgSize = array([640,640,3])

	def pose_callback(self, msg):
		self.curr_pose = msg
		# gets the euler angles (roll, pitch, yaw) from the quaternion values
		# Note: The angles you get doesn't map the [-pi,pi] range properly and might require some conversion
		self.orientation = euler_from_quaternion((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))


	def img_cb(self,msg):
		try:
			if self.curr_pose.pose.position.z > 0:
				# access the visual from 'frame' to get the rover coordinates
				frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
				# add your image processing code here
				self.img = frame

				if self.debug:
					# could be used to debug your detection logic
					data = self.bridge.cv2_to_imgmsg(frame,"bgr8")
					data.header.stamp = msg.header.stamp
					self.debugImgPub.publish(data)
		except CvBridgeError as e:
#			print(e)
			pass

	def state_callback(self, msg):
		if msg.mode != 'OFFBOARD' or self.arm != True:
			# take_off
			self.set_offboard_mode()
			self.set_arm()

	def set_offboard_mode(self):
		rospy.wait_for_service('/uav/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/uav/mavros/set_mode', SetMode)
			isModeChanged = flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

	def set_arm(self):
		rospy.wait_for_service('/uav/mavros/cmd/arming')
		try:
			self.armService = rospy.ServiceProxy('/uav/mavros/cmd/arming', CommandBool)
			self.armService(True)
			self.arm = True
		except rospy.ServiceException as e:
			print("Service arm call failed: %s" % e)



	def attach_models(self, model1, link1, model2, link2):
		req = AttachRequest()
		req.model_name_1 = model1
		req.link_name_1 = link1
		req.model_name_2 = model2
		req.link_name_2 = link2
		self.attach_srv.call(req)

	def detach_models(self, model1, link1, model2, link2):
		req = AttachRequest()
		req.model_name_1 = model1
		req.link_name_1 = link1
		req.model_name_2 = model2
		req.link_name_2 = link2
		self.detach_srv.call(req)


	def retro(self):
		while self.mode == 'RETRO' and not rospy.is_shutdown():
			# using the following line of code to detach the probe
			self.detach_models('if750a_1','base_link','sample_probe','base_link')
			self.mode = "LAND"
			

	def land(self):
		rate = rospy.Rate(15)  # Hz
		while self.mode == "LAND" and not rospy.is_shutdown():
			try:
				flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
				isModeChanged = flightModeService(custom_mode='AUTO.LAND')
			except rospy.ServiceException as e:
				print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	def ascend(self):
		self.des_pose.pose.position.z = 10
		rate = rospy.Rate(15)
		dir0 = None
		dir1 = None
		t2l = 1.25

		d = rospy.Duration.from_sec(3)
		tk = rospy.Time.now()
		while self.mode=="ASCEND" and not rospy.is_shutdown():
			self.pos_setpoint_pub.publish(self.des_pose)
			if self.curr_pose.pose.position.z>0.5 and self.attachFlag:
				# Use the following line of code to attach the probe to the drone...
				self.attach_models('if750a_1','base_link','sample_probe','base_link')
				self.attachFlag = False
			# if the drone is ready for the throw, change mode to "BELLY-FLOP"
			# self.mode = "BELLL-FLOP"
			if self.curr_pose.pose.position.z > 9:
				# self.mode = "BELLY-FLOP"
				mask = cv.inRange(self.img, np.array([0,0,0]), np.array([100,100,100]))
				external_poly = np.array( \
					[[200, 100], [200, 540], [440, 540], [440, 100]], dtype=np.int32)
				cv.fillPoly(mask, pts=[external_poly], color=(0,0,0))
				# cv.imshow("mask", mask)
				# cv.waitKey(1)
				pixels = np.argwhere(mask == 255)
				if len(pixels) > 0:
					avg_x = np.mean(pixels[:,1])
					avg_y = np.mean(pixels[:,0])
					center_x = avg_x - 320
					center_y = -1*(avg_y - 320)
					dir = np.array([center_x, center_y]) / np.sqrt(center_x**2 + center_y**2)
					ang = np.arccos(np.dot(dir, [1,0]))
					if dir[1] < 0:
						ang = 2*np.pi - ang
					# print(dir)
					if not (dir1 is None):
						# print(dir)
						# print(np.pi / 2, ang, t2l*rangle)
						# print(np.abs(2*np.pi - (ang + t2l*rangle)))
						if np.abs((np.pi / 2) - (ang + t2l*rangle)) < 0.1:
							self.mode = "BELLY-FLOP"
							self.d1 = True # true -> vertical
							self.d2 = True # true -> up
						elif np.abs((np.pi) - (ang + t2l*rangle)) < 0.1:
							self.mode = "BELLY-FLOP"
							self.d1 = False # false -> horizontal
							self.d2 = False # false -> left
						elif (np.abs((ang + t2l*rangle)) < 0.1) or (np.abs(2*np.pi - (ang + t2l*rangle)) < 0.1):
							self.mode = "BELLY-FLOP"
							self.d1 = False
							self.d2 = True # true -> right
						elif np.abs((3*np.pi / 2) - (ang + t2l*rangle)) < 0.1:
							self.mode = "BELLY-FLOP"
							self.d1 = True
							self.d2 = False # false -> down
					if dir0 is None and rospy.Time.now() > tk + rospy.Duration.from_sec(1):
						dir0 = dir
						t0 = rospy.Time.now()
					if dir1 is None and rospy.Time.now() > t0 + d:
						dir1 = dir
						td = rospy.Time.now() - t0
						tdf = td.to_sec()
						angle = np.arccos(np.dot(dir0, dir1))
						rangle = angle / tdf
						print(rangle)
						# self.mode = "BELLY-FLOP"
						self.curr_dir = dir
						self.car_rate = rangle
				# self.mode = "SPIN"
			rate.sleep()

	def belly_flop(self):

		# add your flip code here 

		rate = rospy.Rate(30)  # Hz
		self.set_offboard_mode()
		self.att.body_rate = Vector3()
		self.att.header = Header()
		self.att.header.frame_id = "base_footprint"
		self.attach = True

		t0 = rospy.Time.now()
		d = rospy.Duration.from_sec(0.15)
		while self.mode == "BELLY-FLOP" and not rospy.is_shutdown():
			self.att.header.stamp = rospy.Time.now()
			# use AttitudeTarget.thrust to lift your quadcopter
			self.att.thrust = 0.7
			# use AttitudeTarget.body_rate.y to provide the angular velocity to your quadcopter
			if self.d1:
				self.att.body_rate.y = 9.28 if self.d2 else -9.28
			else:
				self.att.body_rate.x = 9.28 if self.d2 else -9.4
			# type_mask = 128 is used for controlling the rate exclusively, you may explore other values too
			self.att.type_mask = 128

			up = self.d1 and self.d2 and self.orientation[1] > 1.2
			right = not self.d1 and self.d2 and self.orientation[0] > 1.2
			down = self.d1 and not self.d2 and self.orientation[1] < -1.2
			left = not self.d1 and not self.d2 and self.orientation[0] < -1.2

			if (up or right or down or left) \
					and rospy.Time.now() > t0 + d:
				print(self.orientation[0], self.orientation[1])
				self.att_setpoint_pub.publish(self.att)
				rate.sleep()
				self.mode = "RETRO"

			# if (you think the drone is ready to detach the probe):
			#     self.att_setpoint_pub.publish(self.att)
			#     rate.sleep()
			#     self.mode="RETRO"

			self.att_setpoint_pub.publish(self.att)

			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass



	def controller(self):
		""" A state machine developed to have UAV states """
		while not rospy.is_shutdown():
			# control your UAV states and functionalities here...
			if self.mode =="ASCEND":
				print("Ascending!")
				self.ascend()
			if self.mode =="BELLY-FLOP":
				print("belly flop!")
				self.belly_flop()
			if self.mode == "RETRO":
				self.retro()
			if self.mode == "LAND":
				self.land()


if __name__ == "__main__":
	
	OffboardControl()