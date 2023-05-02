import rospy
import actionlib
from std_msgs.msg import String
from manipulation_pepper.msg import MovementAction, MovementResult, MovementFeedback
from movement_moveit import MovementMoveit

class MovementActionServer(object):
    def __init__(self):
        self.movement = MovementMoveit()
        self.action_server = actionlib.SimpleActionServer("Movement", MovementAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()

        rospy.loginfo("Movement server started")
      
    def execute_cb(self, goal):
        rospy.loginfo(f"Received order : {goal.order}")

        success=False

        try:
            if(len(goal.order) > 0):

                # Actions with pose arguments

                plan = None

                if(len(goal.target) > 1):

                    rospy.loginfo(f"Received target : {goal.target}.")

                    rounded_target = []
                    for x in goal.target:
                        rounded_target.append(round(x, 8))
                        
                    rospy.loginfo(f"Rounded target : {rounded_target}.")

                    if(goal.order=="plan_joints"):
                        rospy.loginfo("Executing plan joints")
                        plan = self.movement.plan_joints()
                        
                    elif(goal.order=="plan_xyz"):
                        rospy.loginfo("Executing plan xyz")
                        plan = self.movement.plan_xyz_position(goal.target[:3])
                        self.movement.info()

                    elif(goal.order=="plan_rpy"):
                        rospy.loginfo("Executing plan rpy")
                        plan = self.movement.plan_rpy_orientation(goal.target[:3])

                    elif(goal.order=="plan_q"):
                        rospy.loginfo("Executing plan q")
                        plan = self.movement.plan_q_orientation(goal.target[:4])

                    elif(goal.order=="plan_6d"):
                        rospy.loginfo("Executing plan 6d")
                        plan = self.movement.plan_6d_pose(rounded_target)
                    
                    # If a plan was found, execute it
                    if(plan[0] == True):
                        success = self.execute(plan)
                
                elif(goal.order=="set_hand"):
                    rospy.loginfo(f"Received target : {goal.target}.")
                    rospy.loginfo("Executing set hand")
                    self.movement.set_hand(goal.target[0])
                    success=True

                # Actions without arguments

                elif(goal.order=="stop_hand"):
                    rospy.loginfo("Executing stop hand")
                    self.movement.stop_hand()
                    success=True

                elif(goal.order =="pose_middle"):
                    rospy.loginfo("Executing pose middle")
                    self.movement.pose_middle()
                    success=True

                elif(goal.order=="pose_dialog"):
                    rospy.loginfo("Executing pose dialog")
                    self.movement.pose_dialog()
                    success=True

                elif(goal.order=="pose_restaurant"):
                    rospy.loginfo("Executing pose restaurant")
                    self.movement.pose_restaurant()
                    self.action_server.publish_feedback(MovementFeedback(feedback="holding restaurant pose"))
                    success=True

                    # ADD OPEN HAND (use thumb to stabilize tray)

                elif(goal.order=="stop_pose_restaurant"):
                    rospy.loginfo("Executing stop pose restaurant")
                    self.action_server.publish_feedback(MovementFeedback(feedback="stopping restaurant pose"))
                    self.movement.stop_pose_restaurant()
                    self.movement.pose_middle() # Remove later
                    success=True

                elif(goal.order=="raise_hand"):
                    rospy.loginfo("Executing raise hand")
                    self.movement.move_hand_up()
                    success=True

                elif(goal.order=="crouch"):
                    rospy.loginfo("Executing crouch")
                    self.movement.crouch()
                    success=True

                elif(goal.order=="straight_arm"):
                    rospy.loginfo("Executing straight arm")
                    self.movement.straight_arm()
                    success=True

                elif(goal.order=="grab_bag"):
                    rospy.loginfo("Executing grab bag")
                    #self.movement.crouch()
                    self.movement.straight_arm()
                    rospy.sleep(1)
                    self.movement.hold_bag()
                    success=True

                elif(goal.order=="put_down_bag"):
                    rospy.loginfo("Executing grab bag")
                    self.movement.crouch()
                    self.movement.stop_hold_bag()
                    self.movement.straight_arm()
                    rospy.sleep(1)
                    self.movement.hold_bag()
                    success=True

                elif(goal.order=="info"):
                    rospy.loginfo("Executing info")
                    self.movement.info()
                    success=True

                elif(goal.order=="rand_pose"):
                    self.movement.movegroup_rarm.set_start_state_to_current_state()
                    target = self.movement.movegroup_rarm.get_random_pose()
                    plan = self.movement.plan_6d_pose(target)
                    success=plan[0]

                elif(goal.order=="hold_last_pose"):
                    rospy.loginfo("Executing hold_last_pose")
                    self.movement.hold_last_pose()
                    success=True
                
                elif(goal.order=="stop_hold_last_pose"):
                    rospy.loginfo("Executing stop hold_last_pose")
                    self.movement.stop_hold_last_pose()
                    success=True

                elif(goal.order=="grab_2arms"):
                    rospy.loginfo("Executing grab_2arms")
                    self.movement.grab_2arms()
                    success=True

                elif(goal.order=="release_grab_2arms"):
                    rospy.loginfo("Executing release grab_2arms")
                    self.movement.release_grab_2arms()
                    success=True
                
                elif(goal.order=="test"):
                    rospy.loginfo("Executing test")
                    target = self.movement.movegroup_rarm.get_random_pose()
                    plan = self.movement.plan_6d_pose(target)
                    rospy.sleep(1)
                    success = self.execute(plan)

                elif(goal.order=="stop"):
                    rospy.loginfo("Stopping current action")
                    self.movement.stop()
                    success=True

                else:
                    rospy.logerr(f"Unrecognized order : {goal.order}")
                    success=False
            else:
                rospy.logerr("No order")
                success=False
        except (AttributeError,RuntimeError,UnboundLocalError,TypeError) as e:
            rospy.logerr(e)

        result = MovementResult(success)
        rospy.loginfo(f"Sending result : {result}")
        self.action_server.set_succeeded(result)

    def execute(self, plan):
        if(plan[0]):
            success = self.movement.execute_plan(plan[1])
        else:
            success = False
        return success
        
if __name__ == '__main__':
    rospy.init_node('movement')
    server = MovementActionServer()
    rospy.spin()