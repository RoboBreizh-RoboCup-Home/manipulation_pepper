from actionlib import SimpleActionClient
from naoqi_bridge_msgs.msg import JointTrajectoryAction, JointAnglesWithSpeed
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from threading import Thread
import rospy
import time

## Movement class contains all variables and functions for the movements of the Pepper robot

class Movement :

	##Constructor of the Movement class
	#Initialise multiple attributes which are set by default
	#@param self
	def __init__(self):
		rospy.init_node("movement")

		self.client_traj = SimpleActionClient("joint_trajectory",JointTrajectoryAction)
		self.pub_turn = rospy.Publisher("cmd_vel",Twist,queue_size=10)
		self.pub_angles = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)
		while self.pub_turn.get_num_connections()!=0 or self.pub_angles.get_num_connections()!=0 :
			pass
		self.joint_names = ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
		self.dialog = [0.018, -0.382, 1.766, 0.074, -1.720, -0.114, 0.048, 1.740, -0.074, 1.670, 0.104, -0.026]
		self.detection = [0.018, 0.288, 1.766, 0.074, -1.720, -0.114, 0.048, 1.740, -0.074, 1.670, 0.104, -0.026]
		self.middle = [0.018, -0.047, 1.766, 0.074, -1.720, -0.114, 0.048, 1.740, -0.074, 1.670, 0.104, -0.026]

		self.joint_names_handup = ["HeadYaw","HeadPitch", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
		self.handup_joints = [0.018, -0.382, 1.293, -0.350, 1.704, 1.273, 1.539]

		self.joint_rarm = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
		self.joint_larm = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
		self.joint_head = ["HeadYaw","HeadPitch"]
		self.joint_rhand = ["RHand"]
		self.joint_lhand = ["LHand"]

		self.thread_pose_restaurant = None
	##Destructor of a Movement object
	#@param self
	def __del__(self):
		if self.thread_pose_restaurant != None :
			self.thread_pose_restaurant.join()

	##Function to make the robot rise his hand up
	#@param self
	def move_hand_up(self):
		print("thread hand up")
		msg = JointAnglesWithSpeed()
		msg.joint_names = self.joint_names_handup
		msg.joint_angles = self.handup_joints
		msg.speed = 0.2
		#while self.bool_hand_up :
		print(f"Publishing : {msg}")
		self.pub_angles.publish(msg)
		print("end thread hand up")

	def close_hand(self):
		goal = 20
		msg = JointAnglesWithSpeed()
		msg.joint_names = self.joint_rhand
		msg.joint_angles = [np.deg2rad(goal)]
		msg.speed = 0.2
		print(f"Publishing : {msg}")
		self.pub_angles.publish(msg)

	def pose_middle(self):
		msg = JointAnglesWithSpeed()
		msg.joint_names = self.joint_names
		msg.joint_angles = self.middle
		msg.speed = 0.1
		print(f"Publishing : {msg}")
		self.pub_angles.publish(msg)

	def pose_dialog(self):
		msg = JointAnglesWithSpeed()
		msg.joint_names = self.joint_names
		msg.joint_angles = self.dialog
		msg.speed = 0.1
		print(f"Publishing : {msg}")
		self.pub_angles.publish(msg)

	def pose_restaurant_task(self):
		#left = [0.610865,0.0872665,-1.48353,-0.523599,-1.74533] #35/5/-85/-30/-100
		#right = [0.610865,-0.0872665,1.48353,0.523599,1.74533] #35/-5/85/30/100

		left = [0.314159,0.0523599,-0.785398,-0.610865,-1.8326]
		right = [0.314159,-0.0523599,0.785398,0.610865,1.8326] #18/-3/45/35/105

		msgl = JointAnglesWithSpeed()
		msgl.joint_names = self.joint_larm
		msgl.joint_angles = left
		msgl.speed = 0.1
		
		msgr = JointAnglesWithSpeed()
		msgr.joint_names = self.joint_rarm
		msgr.joint_angles = right
		msgr.speed = 0.1

		while(self.holding_pose_restaurant):
			self.pub_angles.publish(msgl)
			self.pub_angles.publish(msgr)
			time.sleep(1)

	def pose_restaurant(self):
		self.holding_pose_restaurant = True
		self.thread_pose_restaurant = Thread(target=self.pose_restaurant_task)
		self.thread_pose_restaurant.start()

	def stop_pose_restaurant(self):
		self.holding_pose_restaurant = False
		self.thread_pose_restaurant.join()