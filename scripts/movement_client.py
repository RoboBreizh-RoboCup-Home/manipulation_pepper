import rospy
import actionlib
import sys
import argparse
from std_msgs.msg import String
from manipulation_pepper.msg import MovementAction, MovementGoal

def movement_client(order):
    client = actionlib.SimpleActionClient('Movement', MovementAction)
    client.wait_for_server()

    goal = MovementGoal(order=order)

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Send an order to the movement server.")
    parser.add_argument("order", help="Order to send (raise_hand, pose_dialog, pose_middle, pose_restaurant, stop_pose_restaurant, plan_joints, plan_6d)")
    args = parser.parse_args()

    if args.order:
        try:
            rospy.init_node('movement_client')
            result = movement_client(args.order)
            print(f"Result : {result}")
        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)