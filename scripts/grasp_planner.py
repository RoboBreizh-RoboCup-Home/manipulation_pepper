import rospy
import time
from sensor_msgs.msg import PointCloud2
from manipulation_pepper.msg import MovementGoal, GraspAction, GraspServerRequest, GraspActionGoal
from manipulation_pepper.srv import object_detection, object_detectionRequest
from gpd_ros.msg import GraspConfigList
import actionlib

class GraspPlanner :
    def __init__(self):
        #rospy.init_node("grasp_planner_server")

        self.action_server = actionlib.SimpleActionServer("GraspPlanner", GraspAction, execute_cb=self.callback_action_server, auto_start = False)
        self.action_server.start()
        
        self.sub_pcl = rospy.Subscriber("/points", PointCloud2, self.callback_pcl) # /cloud_pcd
        self.sub_grasp = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.callback_grasp) # /detect_grasps/plot_grasps

        self.pub_movement = rospy.Publisher("/Movement/goal", MovementGoal, queue_size=10)
        self.pub_pcl = rospy.Publisher("/cloud_stitched", PointCloud2, queue_size=10)

        rospy.wait_for_service('/object_pc_detection')
        self.pointlcoud_service = rospy.ServiceProxy('/object_pc_detection', object_detection)

        self.pcl = None

    def callback_pcl(self,data):
        #rospy.loginfo(f"cb pointcloud")
        self.pcl = data

    def callback_grasp(self,data):
        rospy.loginfo(f"cb grasp : {data}")
        msg = MovementGoal()
        msg.order = "plan_6d"

    def callback_action_server(self, goal):
        rospy.loginfo(f"action server cb with goal : {goal}")

        req = object_detectionRequest()
        #rospy.loginfo(req)
        req.x_min = int(goal.x_min)
        req.x_max = int(goal.x_max)
        req.y_min = int(goal.y_min)
        req.y_max = int(goal.y_max)
        req.pc = self.pcl

        # msg = PointCloud2()

        resp = self.pointlcoud_service(req)

        rospy.loginfo(f"Response from node : {resp.pc.header}")

        msg = PointCloud2()
        msg = resp.pc
        self.pub_pcl.publish(msg)

        self.action_server.set_succeeded(True)

    def plan_grasp(self):
        self.pub_pcl.publish(self.pcl)

if __name__=="__main__":
    rospy.init_node("grasp_planner")
    gp = GraspPlanner()
    rospy.sleep(5)
    pub = rospy.Publisher("/GraspPlanner/goal", GraspActionGoal, queue_size=10)

    try:
        rospy.spin()
    except (Exception, KeyboardInterrupt) as e:
        print(e)
        quit()
