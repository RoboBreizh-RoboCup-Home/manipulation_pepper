import rospy
import actionlib
from std_msgs.msg import String
from manipulation_pepper.msg import MovementAction, MovementResult, MovementFeedback
from movement import Movement

class MovementActionServer(object):
    def __init__(self):
        self.movement = Movement()
        self.action_server = actionlib.SimpleActionServer("Movement", MovementAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()

        rospy.loginfo("Movement server started")
      
    def execute_cb(self, goal):
        rospy.loginfo(f"Received order : {goal.order}")

        success=False

        try:
            if(len(goal.order) > 0):

                if(goal.order=="set_hand"):
                    rospy.loginfo(f"Received target : {goal.target}.")
                    rospy.loginfo("Executing set hand")
                    self.movement.set_hand(goal.target[0])
                    success=True

                # Actions without arguments

                elif(goal.order=="stop_hand"):
                    rospy.loginfo("Executing stop hand")
                    self.movement.stop_hand()
                    success=True

                elif(goal.order=="close_hand"):
                    rospy.loginfo("Executing close hand")
                    self.movement.set_hand(0.0)
                    success=True

                elif(goal.order =="pose_middle"):
                    rospy.loginfo("Executing pose middle")
                    self.movement.pose_middle()
                    success=True

                elif(goal.order=="pose_dialog"):
                    rospy.loginfo("Executing pose dialog")
                    self.movement.pose_dialog()
                    success=True

                elif(goal.order=="pose_pregrasp"):
                    rospy.loginfo("Executing pose pre grasp")
                    self.movement.pose_pregrasp()
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

                elif(goal.order=="pose_straightarm"):
                    rospy.loginfo("Executing straight arm")
                    self.movement.straight_arm()
                    success=True

                elif(goal.order=="pose_getobject"):
                    rospy.loginfo("Executing pose get object")
                    self.movement.pose_getobject()
                    success=True

                elif(goal.order=="pose_release2arms"):
                    rospy.loginfo("Executing pose_release2arms")
                    self.movement.pose_release2arms()
                    self.movement.stop()
                    success=True

                elif(goal.order=="grab_bag"):
                    rospy.loginfo("Executing grab bag")
                    #self.movement.crouch()
                    self.movement.straight_arm()
                    rospy.sleep(1)
                    self.movement.hold_bag()
                    success=True

                elif(goal.order=="grab_right"):
                    rospy.loginfo("Executing grab bag")
                    self.movement.straight_arm()
                    rospy.sleep(1)
                    self.movement.set_hand(0.0)
                    success=True

                elif(goal.order=="put_down_bag"):
                    rospy.loginfo("Executing grab bag")
                    self.movement.crouch()
                    self.movement.stop_hold_bag()
                    self.movement.straight_arm()
                    rospy.sleep(1)
                    self.movement.hold_bag()
                    success=True

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
                    if(goal.target[0] and goal.target[1]):
                        rospy.loginfo("Got 2 values")
                        self.movement.grab_2arms(goal.target[0],goal.target[1])
                    else:
                        rospy.loginfo("Got no values")
                        self.movement.grab_2arms()
                    success=True

                elif(goal.order=="release_grab_2arms"):
                    rospy.loginfo("Executing release grab_2arms")
                    self.movement.release_grab_2arms()
                    success=True
                
                elif(goal.order=="test"):
                    rospy.loginfo("Executing test")
                    rospy.sleep(1)
                    success = True
                    
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
        except Exception as e:
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