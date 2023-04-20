import rospy
import time
from sensor_msgs.msg import PointCloud2
from manipulation_pepper.msg import MovementAction, MovementResult, MovementFeedback
from gpd_ros.msg import GraspConfig, CloudSamples
class GraspPlanner :
    def __init__(self) -> None:
        self.sub_pcl = rospy.Subscriber("/cloud_pcd", PointCloud2, self.callback_pcl)
        self.sub_grasp = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfig, self.callback_grasp) # /detect_grasps/plot_grasps
        
        self.pub_movement = rospy.Publisher("/Movement/goal", MovementAction, queue_size=10)
        self.pub_pcl = rospy.Publisher("/cloud_stitched", CloudSamples, queue_size=10)
        self.pcl = None

    def callback_pcl(self,data):
        rospy.loginfo(f"cb pointcloud : {data}")
        self.pcl = data

    def callback_grasp(self,data):
        rospy.loginfo(f"cb grasp : {data}")
        msg = MovementAction()
        msg.goal.order = "plan_6d"
        
    def plan_grasp(self):
        self.pub_pcl.publish(self.pcl)

if __name__=="__main__":
    rospy.init_node("grasp_planner")
    gp = GraspPlanner()
    pub = rospy.Publisher("/cloud_pcd", PointCloud2, queue_size=10)
    try:
        while True:
            time.sleep(1)
            # msg = PointCloud2()
            # pub.publish(msg)
    except KeyboardInterrupt:
        quit()
