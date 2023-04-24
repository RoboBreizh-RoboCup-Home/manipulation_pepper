#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ["MODEL_PATH"] = "/home/nhan/detection_ws/src/robobreizh/scripts/models"
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
from sensor_msgs.msg import PointCloud2, Image, PointField

import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from manipulation_pepper.srv import object_detection
# from manipulation_pepper.msg import DetectedObj, BoundingBoxCoord

from yolov8 import YOLOv8
from yolov8.utils import draw_bounding_box_opencv
from yolov8.utils import class_names as CLASSES


class DarkNet_YCB():
    def __init__(self):
        self.model = os.path.join(os.environ.get('MODEL_PATH', './'), "yolov8n_ycb.onnx")

        self.cv2_detector = cv2.dnn.readNetFromONNX(self.model)

        self.yolov8_detector = YOLOv8(self.model, conf_thres=0.5, iou_thres=0.5)

       
    def detect_opencv(self, orig_image):
        # time_1 = time.time()

        [height, width, _] = orig_image.shape
        length = max((height, width))
        image = np.zeros((length, length, 3), np.uint8)
        image[0:height, 0:width] = orig_image
        scale = length / 640

        blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640))
        self.cv2_detector.setInput(blob)
        outputs = self.cv2_detector.forward()

        outputs = np.array([cv2.transpose(outputs[0])])
        rows = outputs.shape[1]

        boxes = []
        scores = []
        class_ids = []

        for i in range(rows):
            classes_scores = outputs[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    outputs[0][i][0] - (0.5 * outputs[0][i][2]), outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                    outputs[0][i][2], outputs[0][i][3]]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

        img = orig_image.copy()
        detections = []
        for i in range(len(result_boxes)):
            index = result_boxes[i]
            box = boxes[index]
            # (x1, y1, x2, y2) top-left and bottom-right corners 
            coordinates = (round(box[0] * scale), round(box[1] * scale),
                            round((box[0] + box[2]) * scale), round((box[1] + box[3]) * scale))
            detection = {
                'class_id': class_ids[index],
                'class_name': CLASSES[class_ids[index]],
                'confidence': scores[index],
                'box': box,
                'scale': scale,
                'coordinates': coordinates}
            detections.append(detection)
            img = draw_bounding_box_opencv(orig_image, class_ids[index], scores[index], round(box[0] * scale), round(box[1] * scale),
                            round((box[0] + box[2]) * scale), round((box[1] + box[3]) * scale))

        # time_2 = time.time()
        # print("Detection time OPENCV:", time_2 - time_1)
        # print("Object detected OPENCV: ", detections)
        return img, detections
    
    

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
        # self.model = DarkNet_YCB()
        # rospy.loginfo("FINISH LOAD MODEL")
        
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
        self.pointcloud_sub = message_filters.Subscriber(
            '/points', PointCloud2)
        
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
        pointcloud_sub = req.pc
        
        # Init fields and header for pointcloud publisher
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                 ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = pointcloud_sub.header.frame_id
        try:
            # Crop pointcloud according to bounding box
            cropped_points = self.crop_pointcloud([x_start, y_start, x_end, y_end], pointcloud_sub)
            # Do the pointcloud segmentation
            segmented_points = self.pointcloud_segmentation(np.array(cropped_points))
            # Convert to pointcloud msg
            pc_msg = pc2.create_cloud(header, fields, segmented_points)
            
        except Exception as e:
            rospy.logerr(e)
            # If error occur while cropping the recieved pointcloud,
            ## the empty points will be sent
            pc_msg = pc2.create_cloud(header, fields, [])            
        
        self.publish_point_cloud(pc_msg)
        
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
            [self.image_sub, self.pointcloud_sub], 10, 1, allow_headerless=True)
        ts.registerCallback(self.callback)

        rospy.loginfo("Launching Detection Node")

        rospy.spin()
        

    def callback(self, image_sub, pointcloud_sub):
        """
        Apply planar extraction and clustering to extract object pointcloud
            after object detection based on pointcloud and image msg.
        
        :param (ros.msg - Image) image_sub: The image msg from subcriber
        :param (ros.msg - PointCloud2) poincloud_sub: The pointcloud msg from subcriber
        """
    
        time_begin = rospy.Time.now()
        rospy.loginfo("INSIDE CALLBACK")
        rgb_image = self.bridge.imgmsg_to_cv2(image_sub, "bgr8")
        
        image, detections = self.model.detect_opencv(rgb_image.copy())

        # cv2.imshow('Inference', image)
        # cv2.waitKey(1)
        print("DETECTED OBJECTS: ", detections)
        
        
        # Only show the first object in the list for testing
        if detections:
            rospy.loginfo("DETECT SOMETHING")
            
            # x_start, y_start, x_end, y_end = detections[0]['coordinates']
            rgb_img = self.crop_image(detections[0]['coordinates'], rgb_image)
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
            self.cropped_rgb_image_pub.publish(rgb_msg)
            
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                     ]

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = pointcloud_sub.header.frame_id
            try:
                # Crop pointcloud according to bounding box
                cropped_points = self.crop_pointcloud(detections[0]['coordinates'], pointcloud_sub)
                # Do the pointcloud segmentation
                segmented_points = self.pointcloud_segmentation(np.array(cropped_points))
                # Convert to pointcloud msg
                pc_msg = pc2.create_cloud(header, fields, segmented_points)
                
            except Exception as e:
                rospy.logerr(e)
                # If error occur while cropping the recieved pointcloud,
                ## the empty points will be sent
                pc_msg = pc2.create_cloud(header, fields, [])              
            
            self.publish_point_cloud(pc_msg)
        
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        rospy.loginfo("PROCESSING TIME: " + str(duration.to_sec()) + " secs")
            

    def crop_pointcloud(self, coordinates, pointcloud_sub):
        """
        Crop pointcloud based on object bounding box coordinates
        
        :param (list) coordinates: The top-left and right-bottom points of bouding box
        :param (ros.msg - PointCloud2) poincloud_sub: The pointcloud msg from subcriber
        :return (list) cropped_points: A list of points inside the bounding box
        """
        
        points = []
        min_x, min_y, max_x, max_y = coordinates
        for i in range(min_x, max_x):
            for j in range(min_y, max_y):
                points.append([i, j])

        cropped_points = list(pc2.read_points(pointcloud_sub, field_names=(
            "x", "y", "z"), skip_nans=True, uvs=points))
        
        return cropped_points
    
    
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
        
        # initialize labels array with -1 to indicate noise
        labels = np.full(len(points), -1)

        # initialize cluster index
        cluster_index = 0

        # loop over each point in the dataset
        for i in range(len(points)):
            # check if point has already been assigned to a cluster
            if labels[i] != -1:
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
                if labels[neighbor] == -1:
                    labels[neighbor] = cluster_index
                    sub_neighbors = np.where(np.linalg.norm(points - points[neighbor], axis=1) < eps)[0]
                    if len(sub_neighbors) >= min_samples:
                        neighbors = np.concatenate((neighbors, sub_neighbors))
                j += 1

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
            n /= np.linalg.norm(normal)
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
        
        # labels = self.dbscan(points, 0.5, 10)
        
        # # count the number of points in each cluster
        # cluster_counts = np.bincount(labels[labels != -1])

        # # find the label of the largest cluster
        # largest_cluster_label = np.argmax(cluster_counts)

        # # extract the indices of the largest cluster
        # largest_cluster_indices = np.where(labels == largest_cluster_label)[0]

        # # extract the points in the largest cluster
        # largest_cluster_points = points[largest_cluster_indices]
        # return largest_cluster_points
        
        outliers = self.ransac_plane(points, num_iterations=100, threshold=0.01)
        return outliers


    def publish_point_cloud(self, cloud_in):
        """
        Apply pointcloud transformation to visulize on rviz
        
        :param (ros.msg - PointCloud2) cloud_in: The pointcloud msg needs transforming
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
        self.visualize_detect_pc.publish(cloud_out)


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

    # obj_detection_node.continuous_node()
    obj_detection_node.service_node()
