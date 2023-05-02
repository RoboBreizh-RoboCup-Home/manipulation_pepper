import rospy
import time
from sensor_msgs.msg import PointCloud2
from manipulation_pepper.msg import MovementGoal, GraspAction, GraspServerRequest, GraspActionGoal, BoundingBoxCoord
from manipulation_pepper.srv import object_detection
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

        req = GraspServerRequest()
        bbox = BoundingBoxCoord()

        #rospy.loginfo(req)
        bbox.x_min = int(goal.x_min)
        bbox.x_max = int(goal.x_max)
        bbox.y_min = int(goal.y_min)
        bbox.y_max = int(goal.y_max)
        req.global_cloud = self.pcl
        req.bounding_box = bbox

        try:
            resp = self.pointlcoud_service(goal.x_min, goal.y_min, goal.x_max, goal.y_max)
            # resp = self.pointlcoud_service(198, 149, 307,271)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.action_server.set_succeeded(False)
            return

        rospy.loginfo(f"Response from node : {resp}")

        # msg = PointCloud2()
        # msg = resp.pc
        # self.pub_pcl.publish(msg)

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
