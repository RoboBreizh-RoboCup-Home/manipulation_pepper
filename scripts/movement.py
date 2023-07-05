from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from geometry_msgs.msg import Twist
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

        ##################################
        # ROS 
        ##################################
        try:
            #self.client_traj = SimpleActionClient("joint_trajectory",JointTrajectoryAction)
            self.pub_turn = rospy.Publisher("cmd_vel",Twist,queue_size=10)
            self.pub_angles = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)
        except(AttributeError,RuntimeError,UnboundLocalError,TypeError) as e:
            rospy.logerr(e)

        while self.pub_turn.get_num_connections()!=0 or self.pub_angles.get_num_connections()!=0 :
            pass
        
        # RIGHT ARM
        self.joint_rarm = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]

        # RIGHT HAND
        self.joint_rhand = ["RHand"]

        # LEFT ARM
        self.joint_larm = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]

        # LEFT HAND
        self.joint_lhand = ["LHand"]

        # BOTH ARMS
        self.joint_botharms = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]

        # BODY
        self.joint_upper_body = ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        self.joint_lower_body = ["HipRoll", "HipPitch", "KneePitch"]

        # HEAD
        self.joint_head = ["HeadYaw","HeadPitch"]

        ##################################
        # POSES
        ##################################

        # UPPER BODY
        self.pose_dialog_angles = [0.018, -0.382, 1.766, 0.074, -1.720, -0.114, 0.048, 1.740, -0.074, 1.670, 0.104, -0.026]
        self.pose_detection_angles = [0.018, 0.288, 1.766, 0.074, -1.720, -0.114, 0.048, 1.740, -0.074, 1.670, 0.104, -0.026]
        self.pose_middle_angles = [0.018, -0.047, 1.766, 0.074, -1.720, -0.114, 0.048, 1.740, -0.074, 1.670, 0.104, -0.026]

        # LOWER BODY
        self.crouch_angles = np.deg2rad([-18,-58,-28])
        self.pose_middle_angles_lbody = np.deg2rad([0,0,0])

        #RARM
        self.pose_handup_angles = [1.293, -0.350, 1.704, 1.273, 1.539]
        self.straight_arm_angles = np.deg2rad([5,-5,70,15,105])
        self.take_bag_angles = np.deg2rad([5,-5,70,55,105])
        self.hold_bag_angles = np.deg2rad([40,-5,70,55,105])

        #BOTH ARMS
        self.pose_grab_2arms_l1 = np.deg2rad([-30,20,-2,-43,-82])
        self.pose_grab_2arms_l2 = np.deg2rad([0,20,-2,-43,-82])
        self.pose_grab_2arms_l3 = np.deg2rad([0,10,-2,-43,-82])

        self.pose_grab_2arms_r1 = np.deg2rad([-30,-20,2,43,82])
        self.pose_grab_2arms_r2 = np.deg2rad([0,-20,2,43,82])
        self.pose_grab_2arms_r3 = np.deg2rad([0,-10,2,43,82])

        self.pose_grab_2arms_1 = np.deg2rad([-30,20,0,-45,-82,  #LARM
                                             -30,-20,0,45,82])   #RARM


        ##################################
        # STATES
        ##################################
        self.crouching = False
        self.holding_pose_restaurant = False
        self.holding_bag = False
        self.hand_joint_value = 0.5
        self.maintain_hand_pose = False
        self.last_pose = None
        self.holding_pose = False

        ##################################
        # THREADS
        ##################################
        self.thread_pose_restaurant = None
        self.thread_crouch = None
        self.thread_hold_bag = None
        self.thread_hand = None
        self.thread_hold_last_pose = None

    ##Destructor of a Movement object
    #@param self
    def __del__(self):
        self.stop()

    ##Function to make the robot rise his hand up
    #@param self
    def move_hand_up(self):
        print("thread hand up")
        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_rarm
        msg.joint_angles = self.pose_handup_angles
        msg.speed = 0.2
        #while self.bool_hand_up :
        print(f"Publishing : {msg}")
        self.pub_angles.publish(msg)
        print("end thread hand up")

    ##Function to make the robot close his hand
    #@param self
    def close_hand(self):
        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_rhand
        msg.joint_angles = 1.0
        msg.speed = 0.4
        print(f"Publishing : {msg}")
        self.pub_angles.publish(msg)

    ##Function to set the robot hand to a joint value and maintain it with a Thread. If the Thread is already running, it modifies the hand joint value that the thread is sending.
    #@param self
    #@param hand_joint_value
    def set_hand(self, hand_joint_value):
        rospy.loginfo(f"got jointvalue : {hand_joint_value}")
        assert hand_joint_value >= 0.0 and hand_joint_value <= 1.0

        self.hand_joint_value = hand_joint_value
        self.maintain_hand_pose = True

        if(self.thread_hand==None):
            rospy.loginfo("start thread hand")
            self.thread_hand = Thread(target=self.maintain_hand_task)
            self.thread_hand.start()
    
    ##Function to stop the hand thread
    #@param self
    def stop_hand(self):
        self.maintain_hand_pose = False
        self.thread_hand.join()
        self.thread_hand = None

    ##Thread to maintain the hand at set joint value
    #@param self
    def maintain_hand_task(self):
        msg = JointAnglesWithSpeed()
        msg.joint_names = [self.joint_rhand[0],self.joint_lhand[0]]
        msg.speed = 0.4

        while(self.maintain_hand_pose):
            msg.joint_angles = [self.hand_joint_value,self.hand_joint_value]
            rospy.sleep(0.5)
            self.pub_angles.publish(msg)

    ##Function to put the robot in a base pose
    #@param self
    def pose_middle(self):
        self.stop()
        self.set_hand(0.5)

        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_upper_body
        msg.joint_angles = self.pose_middle_angles
        msg.speed = 0.1
        print(f"Publishing : {msg}")
        self.pub_angles.publish(msg)

        msg.joint_names = self.joint_lower_body
        msg.joint_angles = self.pose_middle_angles_lbody
        print(f"Publishing : {msg}")
        self.pub_angles.publish(msg)

    ##Function to put the robot in a dialog pose
    #@param self
    def pose_dialog(self):
        self.stop()
        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_upper_body
        msg.joint_angles = self.pose_dialog_angles
        msg.speed = 0.1
        print(f"Publishing : {msg}")
        self.pub_angles.publish(msg)

    def pose_pregrasp(self):
        self.stop()
        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_botharms
        msg.joint_angles = self.pose_grab_2arms_1
        msg.speed = 0.1
        print(f"Publishing : {msg}")
        self.pub_angles.publish(msg)

    ##Thread to maintain the robot in a restaurant pose
    #@param self
    def pose_restaurant_task(self):
        #["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        left = np.deg2rad([53.8,7.5,-70.5,-53,1.7])
        right = np.deg2rad([53.8,-7.5,70.5,53,1.7])

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

    ##Function to set the robot in a restaurant pose (starts a thread to maintain the pose)
    #@param self
    def pose_restaurant(self):
        self.stop()
        self.holding_pose_restaurant = True
        self.thread_pose_restaurant = Thread(target=self.pose_restaurant_task)
        self.thread_pose_restaurant.start()

    ##Function to stop the restaurant pose thread
    #@param self
    def stop_pose_restaurant(self):
        self.holding_pose_restaurant = False
        self.thread_pose_restaurant.join()

    ##Function to set the robot in a crouching position (starts a thread to maintain the pose)
    #@param self
    def crouch(self):
        self.crouching = True
        self.thread_crouch = Thread(target=self.crouch_task)
        self.thread_crouch.start()

    ##Function to stop the crouching pose thread and set the robot back straight
    #@param self
    def stop_crouch(self):
        self.crouching = False
        self.thread_crouch.join()

        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_upper_body
        msg.joint_angles = self.pose_middle_angles_lbody
        msg.speed = 0.1
        print(f"Publishing : {msg}")
        self.pub_angles.publish(msg)

    ##Thread to maintain the robot in a crouching pose
    #@param self
    def crouch_task(self):
        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_upper_body
        msg.joint_angles = self.crouch_angles
        msg.speed = 0.1
        while self.crouching:
            self.pub_angles.publish(msg)

    ##Function to make the robot right arm straight
    #@param self
    def straight_arm(self):
        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_rarm
        msg.joint_angles = self.straight_arm_angles
        msg.speed = 0.1
        rospy.loginfo(f"Publishing : {msg}")
        self.pub_angles.publish(msg)
        self.last_pose = msg

    ##Function to make the robot hold the bag (starts a thread to maintain holding)
    #@param self
    def hold_bag(self):
        self.holding_bag = True
        self.thread_hold_bag = Thread(target=self.crouch_task)
        self.thread_hold_bag.start()

    ##Thread to maintain the robot holding a bag
    #@param self
    def hold_bag_task(self):
        msg = JointAnglesWithSpeed()
        msg.joint_names = self.joint_rarm
        msg.joint_angles = self.hold_bag_angles
        msg.speed = 0.1
        while self.crouching:
            self.pub_angles.publish(msg)

    ##Function to stop the bag holding thread
    #@param self
    def stop_hold_bag(self):
        self.holding_bag = False
        self.thread_hold_bag.join()

    def grab_2arms(self,table_height=75,object_width=5):
        self.stop()
        
        spitch = -1.026*table_height+92.31
        sroll = 0.024*object_width**2 + 0.310*object_width + 7.9

        # BASE POSE
        self.pose_pregrasp()
        rospy.sleep(3)

        # GO TOWARDS OBJECT

        msg_twist = Twist()
        msg_twist.linear.x = 0.2
        self.pub_turn(msg_twist)
        rospy.sleep(0.5)
        msg_twist = Twist()
        msg_twist.linear.x = 0.2
        self.pub_turn(msg_twist)


        # LOWER ARMS (ShoulderPitch)
        msg = JointAnglesWithSpeed()
        msg.joint_names = ["RShoulderPitch","LShoulderPitch"]
        msg.joint_angles = np.deg2rad([spitch,spitch])

        rospy.loginfo(f"second msg :\n{msg}")
        self.pub_angles.publish(msg)
        rospy.sleep(1)

        # GRAB WITH ARMS (ShoulderRoll)
        msg.joint_names = ["LShoulderRoll","RShoulderRoll"]
        msg.joint_angles = np.deg2rad([sroll,-1*sroll])

        rospy.loginfo(f"third msg :\n{msg}")
        self.pub_angles.publish(msg)

        msg.joint_names = self.joint_botharms
        msg.joint_angles = self.pose_grab_2arms_1
        [msg.joint_angles[0], msg.joint_angles[5]] = np.deg2rad([spitch, spitch])
        [msg.joint_angles[1], msg.joint_angles[6]] = np.deg2rad([sroll, -1*sroll])
        
        rospy.loginfo(f"HOLD LAST POSE\n{msg}")
        self.last_pose = msg

        self.hold_last_pose()

    def release_grab_2arms(self):
        self.stop_hold_last_pose()
        spitch = 30
        sroll = 25

        # open arms
        msg = JointAnglesWithSpeed()
        msg.joint_names = ["RShoulderRoll","LShoulderRoll"]
        msg.joint_angles = np.deg2rad([-1*sroll,sroll])
        msg.speed = 0.1

        self.pub_angles.publish(msg)
        rospy.loginfo(msg)
        rospy.sleep(1)

        # RAISE ARMS (ShoulderPitch)
        msg.joint_names = self.joint_botharms
        msg.joint_angles = self.pose_grab_2arms_1
        self.pub_angles.publish(msg)
        rospy.loginfo(msg)
        rospy.sleep(1)
        

    ##Function to make the robot hold the last registered pose (starts a thread)
    #@param self
    def hold_last_pose(self):
        self.holding_pose = True
        self.thread_hold_last_pose = Thread(target=self.hold_last_pose_task)
        self.thread_hold_last_pose.start()

    ##Thread to maintain the robot holding last registered pose
    #@param self
    def hold_last_pose_task(self):
        if self.last_pose != None :
            rospy.loginfo("Holding last pose")
            while self.holding_pose:
                self.pub_angles.publish(self.last_pose)
                rospy.sleep(0.5)

    ##Function to stop the bag holding thread
    #@param self
    def stop_hold_last_pose(self):
        rospy.loginfo("Stopped holding last pose")
        self.holding_pose = False
        if self.thread_hold_last_pose != None :
            self.thread_hold_last_pose.join()
    
    ##Function to stop all threads.
    #@param self
    def stop(self):
        self.stop_hold_last_pose()
        if self.crouching:
            self.crouching = False
            self.thread_crouch.join()
        if self.holding_bag:
            self.holding_bag = False
            self.thread_hold_bag.join()
        if self.maintain_hand_pose:
            self.maintain_hand_pose = False
            self.thread_hand.join()
            self.thread_hand = None
        if self.holding_pose_restaurant:
            self.holding_pose_restaurant = False
            self.thread_pose_restaurant.join()
