#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import time
import rospy
import numpy as np

import message_filters
from cv_bridge import CvBridge
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Header, Int64
from sensor_msgs.msg import PointCloud2, Image, PointField, CameraInfo

import tf
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from manipulation_pepper.srv import object_detection
# from manipulation_pepper.msg import DetectedObj, BoundingBoxCoord


from yolov8.yolov8_seg import YOLOSeg
from yolov8.utils import class_names as CLASSES

    
    

class ObjectPointDetection():
    """
    A class used to detect object pointcloud.
    """
    
    def __init__(self):
        """
        Init node for object point cloud detection.
        Init the publishers, subcribers and necessary functions for pointcloud segmentation
        """
        
        rospy.loginfo("INIT NODE")
        rospy.init_node('pointcloud_processing_node', anonymous=False)
    
        # Init Yolo V8 model
        ## Not doing object detection for now
        model_path = "/home/nhan/detection_ws/src/manipulation_pepper/scripts/segment.onnx"
        self.model = YOLOSeg(model_path)
        rospy.loginfo("FINISH LOAD MODEL")
        
        self.bridge = CvBridge() # For handle cv2 image
        # For visualize processed point cloud on rviz
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12)) 
        
        
        # PUBLISHERS
        self.visualize_detect_pc = rospy.Publisher(
            '/visualize_pointcloud', PointCloud2, queue_size=10)
        
        # The following publisher and subcribers are not used for service node
        self.object_pc_pub = rospy.Publisher(
            "/cropped_pointcloud", PointCloud2, queue_size=10)
        self.cropped_depth_image_pub = rospy.Publisher(
            "/cropped_depth_image", Image, queue_size=10)
        self.cropped_rgb_image_pub = rospy.Publisher(
            "/cropped_rgb_image", Image, queue_size=10)

        # SUBSCRIBERS
        self.image_sub = message_filters.Subscriber(
            '/naoqi_driver/camera/front/image_raw', Image)
        self.depth_sub = message_filters.Subscriber(
            '/naoqi_driver/camera/depth/image_raw', Image)
        
        rospy.loginfo("FINISH INIT")


    def service_node(self):
        """
        Service node for object pointcloud detection
        """
        
        s = rospy.Service('object_pc_detection', object_detection,
                          self.handle_pointcloud)
        rospy.loginfo("Object Pointcloud Detection Service Node: Waiting for Request...")
        rospy.spin()


    def handle_pointcloud(self, req):
        """
        Apply planar extraction and clustering to extract object pointcloud
            from input points and bounding box coordinates.
        
        :param (ros.srv - object_detection) req: The request consists of 
                bouding box coordinates and pointcloud from sensor
        :return (ros.msg - PointCloud2) pc_msg: The object pointcloud as PointCloud2 message
        """
        
        time_begin = rospy.Time.now()
        rospy.loginfo("HANDLE REQUEST")
        
        # Bouding box coordinate: top-left and right-bottom points    
        x_start, y_start, x_end, y_end = req.x_min, req.y_min, req.x_max, req.y_max
        depth_sub = rospy.wait_for_message('/naoqi_driver/camera/depth/image_raw', Image)
        depth_image = self.bridge.imgmsg_to_cv2(depth_sub, "16UC1")
        depth_img = self.crop_image([x_start, y_start, x_end, y_end], depth_image)
        
        cameraInfo = rospy.wait_for_message('/naoqi_driver/camera/depth/camera_info', CameraInfo)
        # Generate point cloud from depth image4
        ## Get camera parameter
        cx = cameraInfo.K[2]
        cy = cameraInfo.K[5]
        fx = cameraInfo.K[0]
        fy = cameraInfo.K[4]
        
        cropped_points = []
        height, width = depth_img.shape[:2]
        for v in range(height):
            for u in range(width):
                z = depth_image[v, u] / 1000.0 # Convert depth from millimeters to meters
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                cropped_points.append([x, y, z])
        
        # Init fields and header for pointcloud publisher
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                 ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = cameraInfo.header.frame_id
        
        # Do the pointcloud segmentation
        segmented_points = self.pointcloud_segmentation(np.array(cropped_points))
        # Convert to pointcloud msg
        pc_msg = pc2.create_cloud(header, fields, segmented_points)

        pc_msg = self.transform_pointcloud(pc_msg)
        self.visualize_detect_pc.publish(pc_msg)
        
        # Calculate processing time 
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        rospy.loginfo("PROCESSING TIME: " + str(duration.to_sec()) + " secs")
        
        return pc_msg
        

    def continuous_node(self):
        """
        Publishing node for testing object detection 
            and pointcloud segmentation at the same node 
        """
        
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], 10, 1, allow_headerless=True)
        ts.registerCallback(self.callback)

        rospy.spin()
        

    def callback(self, image_sub, depth_sub):
        """
        Apply planar extraction and clustering to extract object pointcloud
            after object detection based on pointcloud and image msg.
        
        :param (ros.msg - Image) image_sub: The image msg from subcriber
        :param (ros.msg - Image) depth_sub: The depth image msg from subcriber
        """
    
        time_begin = rospy.Time.now()
        rospy.loginfo("INSIDE CALLBACK")
        rgb_image = self.bridge.imgmsg_to_cv2(image_sub, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_sub, "16UC1")
        
        detections = self.model(rgb_image)
        
        
        # Only show the first object in the list for testing
        if len(detections[0]) != 0:
            rospy.loginfo("DETECT SOMETHING")
            
            combined_img = self.model.draw_masks(rgb_image)
            
            rgb_msg = self.bridge.cv2_to_imgmsg(combined_img, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
            self.cropped_rgb_image_pub.publish(rgb_msg)
            self.cropped_depth_image_pub.publish(depth_msg)
            
            
            # cameraInfo = rospy.wait_for_message('/naoqi_driver/camera/depth/camera_info', CameraInfo)
            # # Generate point cloud from depth image4
            # ## Get camera parameter
            # cx = cameraInfo.K[2]
            # cy = cameraInfo.K[5]
            # fx = cameraInfo.K[0]
            # fy = cameraInfo.K[4]
            
            # cropped_points = []
            # height, width = depth_image.shape[:2]
            # for v in range(height):
            #     for u in range(width):
            #         z = depth_image[v, u] / 1000.0 # Convert depth from millimeters to meters
            #         x = (u - cx) * z / fx
            #         y = (v - cy) * z / fy
            #         cropped_points.append([x, y, z])
            
            # fields = [PointField('x', 0, PointField.FLOAT32, 1),
            #           PointField('y', 4, PointField.FLOAT32, 1),
            #           PointField('z', 8, PointField.FLOAT32, 1),
            #          ]

            # header = Header()
            # header.stamp = rospy.Time.now()
            # header.frame_id = cameraInfo.header.frame_id

            # # Do the pointcloud segmentation
            # segmented_points = self.pointcloud_segmentation(np.array(cropped_points))
            # # Convert to pointcloud msg
            # pc_msg = pc2.create_cloud(header, fields, segmented_points)
    
            # pc_msg = self.transform_pointcloud(pc_msg)
            # self.visualize_detect_pc.publish(pc_msg)
        
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        rospy.loginfo("PROCESSING TIME: " + str(duration.to_sec()) + " secs")
    
    
    def dbscan(self, points, eps, min_samples):
        """
        DBScan algorithm for pointcloud clustering (segmentation)
        
        :param (numpy.array) points: The the list of pointcloud
        :param (float) eps: The maximum distance between two samples
                for one to be considered as in the neighborhood of the other
        :param (int) min_samples: The number of samples (or total weight) in a neighborhood 
                for a point to be considered as a core point
        :return (numpy.array) labels: A list of cluster lables of input pointcloud
        """
        
        # initialize labels array with -2 to indicate unvisited
        labels = np.full(len(points), -2)

        # initialize cluster index
        cluster_index = 0

        # loop over each point in the dataset
        for i in range(len(points)):
            # check if point has already been assigned to a cluster
            if labels[i] != -2:
                continue

            # find neighboring points within radius eps
            neighbors = np.where(np.linalg.norm(points - points[i], axis=1) < eps)[0]

            # check if the number of neighbors is greater than min_samples
            if len(neighbors) < min_samples:
                labels[i] = -1  # mark as noise
                continue

            # assign new cluster label to current point
            labels[i] = cluster_index

            # loop over neighboring points and recursively add them to the same cluster
            j = 0
            while j < len(neighbors):
                neighbor = neighbors[j]
                j += 1
                
                if labels[neighbor] not in [-1, -2]:
                    continue
                
                labels[neighbor] = cluster_index
                if labels[neighbor] == -2:
                    sub_neighbors = np.where(np.linalg.norm(points - points[neighbor], axis=1) < eps)[0]
                    if len(sub_neighbors) >= min_samples:
                        neighbors = np.concatenate((neighbors, sub_neighbors))

            # move to the next cluster index
            cluster_index += 1

        return labels


    def ransac_plane(self, points, num_iterations=1000, threshold=0.01):
        """
        RANSAC algorithm for planar extraction
        
        :param (numpy.array) points: The the list of pointcloud
        :param (int) num_iterations: The maximum iterations to find the plane
        :param (float) threshold: The maximum distance between two samples
                for one to be considered as in the plane
        :return (numpy.array) outliers: A list of points outside the plane
        """
        
        # best_plane = None
        best_inliers = None # idx of inliers
        best_num_inliers = 0

        for _ in range(num_iterations):
            # Randomly sample points to form a candidate plane
            sample = points[np.random.choice(points.shape[0], size=3, replace=False), :]

            # Compute the parameters of the candidate plane
            v1 = sample[1] - sample[0]
            v2 = sample[2] - sample[0]
            normal = np.cross(v1, v2) 
            n = normal / np.linalg.norm(normal)
            d = -np.dot(n, sample[0])

            # Compute the distance of each point to the plane
            distances = np.abs(np.dot(points, n) + d) / np.linalg.norm(n)

            # Count the number of inliers
            inliers = np.where(np.abs(distances) < threshold)
            num_inliers = len(inliers)

            # Update the best plane if this one has more inliers
            if num_inliers > best_num_inliers:
                # best_plane = (n, d)
                best_inliers = inliers
                best_num_inliers = num_inliers

        
        # Compute the remaining points that are not on the plane
        mask = np.ones(len(points), dtype=bool)
        mask[best_inliers] = False
        outliers = points[mask]
        
        return outliers


    def pointcloud_segmentation(self, points):
        """
        Extract object pointcloud from cropped pointcloud
        
        :param (numpy.array) points: The the list of cropped pointcloud
        :return (numpy.array) outliers: Object pointcloud after segmentation
        """
        
        # Use ransac to remove planar
        outliers = self.ransac_plane(points, num_iterations=100, threshold=0.01)
        
        # Use dbscan for clustering and keep the largest cluster
        labels = self.dbscan(outliers, 0.008, 8)
        # count the number of points in each cluster
        cluster_counts = np.bincount(labels[labels != -1])
        if len(cluster_counts) == 0:
           return outliers
        # find the label of the largest cluster
        largest_cluster_label = np.argmax(cluster_counts)
        # extract the indices of the largest cluster
        largest_cluster_indices = np.where(labels == largest_cluster_label)[0]
        # extract the points in the largest cluster
        largest_cluster_points = points[largest_cluster_indices]
        
        return largest_cluster_points


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


    def crop_image(self, coordinates, image):
        """
        Crop image based on object bounding box coordinates
        
        :param (list) coordinates: The top-left and right-bottom points of bouding box
        :param (cv2.image) image: The cv2 image need cropping
        :return (cv2.image) cropped_image: The cropped image
        """
        
        min_x, min_y, max_x, max_y = coordinates
        cropped_image = image[min_y:max_y, min_x:max_x]
        return cropped_image

    



if __name__ == "__main__":
    obj_detection_node = ObjectPointDetection()

    # Here you can switch between two mode:
    # 1. Continuous detection by ROS subscriber/callback (asynchronous)
    # 2. Synchronous detection via ROS Service (Server/Client-like)

    obj_detection_node.continuous_node()
    # obj_detection_node.service_node()