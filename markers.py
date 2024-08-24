#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import cv2
from cv_bridge import CvBridge, CvBridgeError

from numpy import isnan, float64, sqrt

import tf2_ros
import tf2_geometry_msgs 

def verify_objects(list_objects, detection):
    limiar = 1

    if len(list_objects) == 0:
        new_object = ObjectMap()
        new_object.coordinates = detection
        list_objects.append(new_object)
    else:
        find_object = False
        for object in list_objects:
            x_pose = detection.point.x
            y_pose = detection.point.y
            z_pose = detection.point.z

            distance = sqrt((object.coordinates.point.x - x_pose)**2 + (object.coordinates.point.y - y_pose)**2 + (object.coordinates.point.z - z_pose)**2)

            if distance < limiar:
                find_object = True
                object.counter += 1
                break 
        
        if not find_object:
            new_object = ObjectMap()
            new_object.coordinates = detection
            list_objects.append(new_object)
    
    return list_objects



class ObjectMap():
    def __init__(self):
        self.counter = 0
        self.coordinates = PointStamped()

class DepthBoundingBox(Node):

    def __init__(self):
        super().__init__('depth_bounding_box')  # Inicialização do nó
        
        self.pose_detections = []

        # Subscriber do tópico de Bounding Box
        self.subscription = self.create_subscription(
            PointStamped,
            "yolov5_ros2/object_pose",
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher das coordenadas de detecção
        self.obj_pose_pub = self.create_publisher(
            MarkerArray,
            "/visualization_marker", 
            10)
        self.obj_pose_pub  # prevent unused variable warning

        self.list_detect_object = []

    def pose_callback(self, msg):
        #print(msg)
        self.list_detect_object = verify_objects(self.list_detect_object, msg)
        self.mark_array = MarkerArray()
        for i in range(len(self.list_detect_object)):
            print(self.list_detect_object[i].coordinates.point, end=' ')
            print(self.list_detect_object[i].counter)

            if self.list_detect_object[i].counter > 50:
                x_pose = self.list_detect_object[i].coordinates.point.x
                y_pose = self.list_detect_object[i].coordinates.point.y
                z_pose = self.list_detect_object[i].coordinates.point.z

                marker = Marker()

                marker.header.frame_id = "/map"
                marker.header.stamp = msg.header.stamp

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = 2
                marker.id = 0

                # Set the scale of the marker
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5

                # Set the color
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # Set the pose of the marker
                marker.pose.position.x = x_pose
                marker.pose.position.y = y_pose
                marker.pose.position.z = z_pose
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0 

                marker.id = i

                self.mark_array.markers.append(marker)
        self.obj_pose_pub.publish(self.mark_array)
        '''
        if not isnan(msg.point.x):
            x_pose = msg.point.x
            y_pose = msg.point.y
            z_pose = msg.point.z

            marker = Marker()

            marker.header.frame_id = "/map"
            marker.header.stamp = msg.header.stamp

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = 2
            marker.id = 0

            # Set the scale of the marker
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Set the color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Set the pose of the marker
            marker.pose.position.x = x_pose
            marker.pose.position.y = y_pose
            marker.pose.position.z = z_pose
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            self.obj_pose_pub.publish(marker)
        '''
        


def main(args=None):
    rclpy.init(args=args)

    depth_boundingbox = DepthBoundingBox()

    rclpy.spin(depth_boundingbox)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_boundingbox.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
