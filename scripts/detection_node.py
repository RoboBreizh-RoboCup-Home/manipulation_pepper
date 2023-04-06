#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ["MODEL_PATH"] = "/home/nhan/detection_ws/src/robobreizh/scripts/models"
import cv2
import time
import rospy
import numpy as np
# import open3d as o3d
# from sklearn.cluster import DBSCAN
# from sklearn.preprocessing import StandardScaler

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
from manipulation_pepper.msg import DetectedObj, BoundingBoxCoord


# import pcl
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
    
    


class ObjectsDetection():
    def __init__(self):
        rospy.loginfo("INIT NODE")
        rospy.init_node('ObjectsDetectionNode', anonymous=False)
    
        # Init Yolo V8 model
        self.model = DarkNet_YCB()

        rospy.loginfo("FINISH LOAD MODEL")
        
        self.bridge = CvBridge()
        self.cloud_points = []
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        
        
        # PUBLISHERS
        self.visualize_detect_pub = rospy.Publisher(
            '/visualize_pointcloud', PointCloud2, queue_size=10)
        
        # Not use for service node
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
        s = rospy.Service('object_detection', object_detection,
                          self.handle_object_detection)
        rospy.loginfo("Object Detection Service Node: Waiting for Request...")
        rospy.spin()


    def handle_object_detection(self, req):
        time_begin = rospy.Time.now()
        rospy.loginfo("HANDLE REQUEST")
            
        x_start, y_start, x_end, y_end = req.x_min, req.y_min, req.x_max, req.y_max
        pointcloud_sub = req.pc
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = pointcloud_sub.header.frame_id
        try:
            self.crop_pointcloud(x_start, y_start, x_end, y_end, pointcloud_sub)
            segmented_points = self.pointcloud_segmentation(np.array(self.cloud_points))
            
            pc_msg = pc2.create_cloud(header, fields, segmented_points)
            
        except Exception as e:
            rospy.logerr(e)

            pc_msg = pc2.create_cloud(header, fields, self.cloud_points)            
        
        self.visualize_detect_pub(pc_msg)
    
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        rospy.loginfo("PROCESSING TIME: " + str(duration.to_sec()) + " secs")
        
        return pc_msg
        

    def continuous_node(self):
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub], 10, 1, allow_headerless=True)
        ts.registerCallback(self.callback)

        rospy.loginfo("Launching Detection Node")

        rospy.spin()
        

    def callback(self, image_sub, pointcloud_sub):
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
            
            x_start, y_start, x_end, y_end = detections[0]['coordinates']
            rgb_img = self.crop_image(x_start, y_start, x_end, y_end, rgb_image)
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
                self.crop_pointcloud(x_start, y_start, x_end, y_end, pointcloud_sub)
                segmented_points = self.pointcloud_segmentation(np.array(self.cloud_points))
             
                pc_msg = pc2.create_cloud(header, fields, segmented_points)
                
            except Exception as e:
                rospy.logerr(e)
                
                pc_msg = pc2.create_cloud(header, fields, self.cloud_points)            
            
            self.publish_point_cloud(pc_msg)
        
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        rospy.loginfo("PROCESSING TIME: " + str(duration.to_sec()) + " secs")
            

    def crop_pointcloud(self, min_x, min_y, max_x, max_y, pointcloud_sub):
        points = []
        for i in range(min_x, max_x):
            for j in range(min_y, max_y):
                points.append([i, j])

        points = list(pc2.read_points(pointcloud_sub, field_names=(
            "x", "y", "z"), skip_nans=True, uvs=points))

        # fields = [PointField('x', 0, PointField.FLOAT32, 1),
        # 	PointField('y', 4, PointField.FLOAT32, 1),
        # 	PointField('z', 8, PointField.FLOAT32, 1),
        # 	PointField('rgb', 16, PointField.FLOAT32, 1),
        # ]

        # header = Header()
        # pc_msg = pc2.create_cloud(header, fields, points)

        # pc_msg.header.stamp = rospy.Time.now()
        # pc_msg.header.frame_id = "/naoqi_driver/camera/front/image_raw"
        self.cloud_points = points
        # self.object_pc_pub.publish(pc_msg)
        
    
    def dbscan(self, points, eps, min_samples):
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


    def ransac_plane(self, points, num_iterations=50, sample_size=3, threshold=0.01):
        best_plane = None
        best_inliers = None
        best_num_inliers = 0

        for i in range(num_iterations):
            # Randomly sample points to form a candidate plane
            sample = points[np.random.choice(points.shape[0], size=sample_size, replace=False), :]

            # Compute the parameters of the candidate plane
            v1 = sample[1] - sample[0]
            v2 = sample[2] - sample[0]
            n = np.cross(v1, v2)
            d = -np.dot(n, sample[0])

            # Compute the distance of each point to the plane
            distances = np.abs(np.dot(points, n) + d) / np.sqrt(np.sum(n**2))

            # Count the number of inliers
            inliers = points[distances < threshold, :]
            num_inliers = inliers.shape[0]

            # Update the best plane if this one has more inliers
            if num_inliers > best_num_inliers:
                best_plane = (n, d)
                best_inliers = inliers
                best_num_inliers = num_inliers

        # Compute the remaining points that are not on the plane
        outliers = points[distances >= threshold, :]

        return outliers, best_inliers


    def pointcloud_segmentation(self, points):
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
        
        outliers, inliers = self.ransac_plane(points, num_iterations=100, threshold=0.01)
        return outliers


    def publish_point_cloud(self, cloud_in):
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
        self.object_pc_pub.publish(cloud_out)


    def convertBack(self, x, y, w, h):
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax


    def crop_image(self, min_x, min_y, max_x, max_y, image):
        image_cropped = image[min_y:max_y, min_x:max_x]
        return image_cropped

    
    def pointcloud_segmentation_o3d(self, points_list):
        # # Normalisation:
        # scaled_points = StandardScaler().fit_transform(points)
        # # Clustering:
        # model = DBSCAN(eps=0.15, min_samples=10)
        # model.fit(scaled_points)
        
        # convert list of points to numpy array
        points_arr = np.array(points_list)

        # create PointCloud object
        pcd = o3d.geometry.PointCloud()

        # assign points to the PointCloud object
        pcd.points = o3d.utility.Vector3dVector(points_arr)
        labels = np.array(pcd.cluster_dbscan(eps=0.1, min_points=10, print_progress=False))

        # get the number of clusters and their sizes
        max_label = labels.max()
        sizes = np.zeros(max_label + 1)
        for i in range(len(labels)):
            sizes[labels[i]] += 1

        # get the index of the largest cluster
        largest_cluster_idx = np.argmax(sizes)

        # extract the points belonging to the largest cluster
        largest_cluster_points = pcd.select_by_index(np.where(labels == largest_cluster_idx)[0])
        return largest_cluster_points
    



if __name__ == "__main__":
    obj_detection_node = ObjectsDetection()

    # Here you can switch between two mode:
    # 1. Continuous detection by ROS subscriber/callback (asynchronous)
    # 2. Synchronous detection via ROS Service (Server/Client-like)

    # obj_detection_node.continuous_node()
    obj_detection_node.service_node()