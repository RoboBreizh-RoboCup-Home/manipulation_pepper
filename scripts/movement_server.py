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
        rospy.loginfo(f"Received order {goal.order}.")
        
        #rospy.sleep(1)
        
        if(goal.order =="pose_middle"):
            rospy.loginfo("Executing pose middle")
            self.movement.pose_middle()
            result = MovementResult(success=True)
        elif(goal.order=="pose_dialog"):
            rospy.loginfo("Executing pose dialog")
            self.movement.pose_dialog()
            result = MovementResult(success=True)
        elif(goal.order=="pose_restaurant"):
            rospy.loginfo("Executing pose restaurant")
            self.movement.pose_restaurant()
            self.action_server.publish_feedback(MovementFeedback(feedback="holding restaurant pose"))
            result = MovementResult(success=True)
        elif(goal.order=="stop_pose_restaurant"):
            rospy.loginfo("Executing stop pose restaurant")
            self.action_server.publish_feedback(MovementFeedback(feedback="stopping restaurant pose"))
            self.movement.stop_pose_restaurant()
            self.movement.pose_middle() # Remove later
            result = MovementResult(success=True)
        elif(goal.order=="raise_hand"):
            rospy.loginfo("Executing raise hand")
            self.movement.move_hand_up()
            result = MovementResult(success=True)
        else:
            rospy.logwarn(f"Unrecognized order : {goal.order}")
            result = MovementResult(success=False)

        rospy.loginfo(f"Sending result : {result}")
        self.action_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('movement')
    server = MovementActionServer()
    rospy.spin()