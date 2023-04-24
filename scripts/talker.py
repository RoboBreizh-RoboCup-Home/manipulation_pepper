 #!/usr/bin/env python
   
import rospy
from std_msgs.msg import String
import tf2_ros
import tf2_py as tf2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Int64
from sensor_msgs.msg import PointCloud2
from manipulation_pepper.srv import object_detection



def publish_point_cloud():
    """
    Apply pointcloud transformation to visulize on rviz
    
    :param (ros.msg - PointCloud2) cloud_in: The pointcloud msg needs transforming
    """
    rospy.init_node('talker', anonymous=True)
    visualize_detect_pc = rospy.Publisher(
            '/test_visualize_pointcloud', PointCloud2, queue_size=10)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
    
    while not rospy.is_shutdown():
        rospy.wait_for_service('object_pc_detection')
        try:
            pc_detect = rospy.ServiceProxy('object_pc_detection', object_detection)
            pc = pc_detect(198, 149, 307,271)
        #    pc = pc_detect(349, 114, 450, 245)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        cloud_in = pc.pc
        
        target_frame = cloud_in.header.frame_id
        source_frame = cloud_in.header.frame_id
        try:
            trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        
        cloud_out = do_transform_cloud(cloud_in, trans)
    
        rospy.loginfo("Visualize pointcloud")
        visualize_detect_pc.publish(cloud_out)



if __name__ == '__main__':
    try:
        publish_point_cloud()
    except rospy.ROSInterruptException:
        pass