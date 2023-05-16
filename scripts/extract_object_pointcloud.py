#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image, PointField, CameraInfo

import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from manipulation_pepper.srv import object_detection


class ExtractObjectPoints():
    """
    A class used to detect object pointcloud.
    """
    
    def __init__(self, VISUAL): 
        self.VISUAL = VISUAL
        self.bridge = CvBridge() # For handle cv2 image
        # For visualize processed point cloud on rviz
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12)) 
        
        
        # PUBLISHERS
        if self.VISUAL:
            self.visualize_detect_pc = rospy.Publisher(
                '/visualize_pointcloud', PointCloud2, queue_size=10)
            
        self.initExtractObjectPointsService()


    def initExtractObjectPointsService(self):
        """
        Service node for object pointcloud extraction
        """
        
        rospy.Service('/robobreizh/manipulation_pepper/extract_object_pointcloud', 
                          object_detection,self.handle_ServiceObjectPointcloud)
        rospy.loginfo("Starting Object Pointcloud Extraction: Waiting for Request...")
        rospy.spin()


    def handle_ServiceObjectPointcloud(self, req):
        """
        Extract object pointcloud from depth image using 
            object mask and bounding box coordinates.
        
        :param (ros.srv - object_detection) req: The request consists of 
                bouding box coordinates and object mask
        :return (ros.msg - PointCloud2) pc_msg: The object pointcloud as PointCloud2 message
        """
        
        time_begin = rospy.Time.now()
        rospy.loginfo("HANDLE REQUEST")
        
        # Bouding box coordinate: top-left and right-bottom points  
        bounding_box = req.bounding_box
        x_start, y_start = bounding_box.top_left.x, bounding_box.top_left.y, 
        x_end, y_end = bounding_box.bottom_right.x, bounding_box.bottom_right.y
        
        depth_sub = rospy.wait_for_message('/naoqi_driver/camera/depth/image_raw', Image)
        depth_image = self.bridge.imgmsg_to_cv2(depth_sub, "16UC1")
        
        rows, cols = req.object_mask.layout.dim[0].size, req.object_mask.layout.dim[1].size
        mask =  np.reshape(req.object_mask.data, (rows, cols))
        crop_mask = mask[y_start:y_end, x_start:x_end] == 1
        crop_depth_img = depth_image[y_start:y_end, x_start:x_end]
        crop_depth_img = crop_depth_img * crop_mask
        
        # Generate point cloud from depth image
        ## Get camera parameter
        cameraInfo = rospy.wait_for_message('/naoqi_driver/camera/depth/camera_info', CameraInfo)
        cx = cameraInfo.K[2]
        cy = cameraInfo.K[5]
        fx = cameraInfo.K[0]
        fy = cameraInfo.K[4]
        
        # Create new depth image to keep the position of object points
        height, width = depth_image.shape[:2]
        new_depth_img = np.zeros((height, width))
        new_depth_img[y_start:y_end, x_start:x_end] = crop_depth_img
        points = []
        for v in range(height):
            for u in range(width):
                z = new_depth_img[v, u] / 1000.0 # Convert depth from millimeters to meters
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])
        
        # Init fields and header for pointcloud publisher
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                 ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = cameraInfo.header.frame_id
    
        # Convert to pointcloud msg
        pc_msg = pc2.create_cloud(header, fields, points)
        pc_msg = self.transform_pointcloud(pc_msg) # Transform to points to match camera frame
        if self.VISUAL:
            self.visualize_detect_pc.publish(pc_msg)
        
        # Calculate processing time 
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        rospy.loginfo("PROCESSING TIME: " + str(duration.to_sec()) + " secs")
        
        return pc_msg


    def transform_pointcloud(self, cloud_in):
        """
        Apply pointcloud transformation to visulize on rviz
        
        :param (ros.msg - PointCloud2) cloud_in: The pointcloud msg needs transforming
        :return (ros.msg - PointCloud2) cloud_out: The transformed pointcloud
        """
        
        target_frame = cloud_in.header.frame_id
        source_frame = cloud_in.header.frame_id
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        
        cloud_out = do_transform_cloud(cloud_in, trans)
        return cloud_out




if __name__ == "__main__":
    VISUAL = rospy.get_param('~visualize')
    rospy.init_node('pointcloud_processing_node', anonymous=True)
    ExtractObjectPoints(VISUAL)