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

from std_msgs.msg import String
from boundingboxes.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import PointStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

from numpy import isnan, float64

import tf2_ros
import tf2_geometry_msgs 

class DepthBoundingBox(Node):

    def __init__(self):
        super().__init__('depth_bounding_box')  # Inicialização do nó

        self.bridge = CvBridge()  # Bridge para converter a depth

        self.depth_info = Image()  # Inicialização de variaveis
        self.depth_image = []

        # Subscriber do tópico de Bounding Box
        self.subscription = self.create_subscription(
            BoundingBoxes,
            '/yolov5_ros2/bounding_boxes',
            self.bbox_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Subscriber do tópico de Depth da Zed
        self.depth = self.create_subscription(
            Image,
            '/zedm/zed_node/depth/depth_registered',
            self.depth_callback,
            10)
        self.depth  # prevent unused variable warning

        # Publisher das coordenadas de detecção
        self.obj_pose_pub = self.create_publisher(
            PoseArray,
            "yolov5_ros2/object_pose", 
            10)
        self.obj_pose_pub  # prevent unused variable warning

        # Zedm camera pinhole model for left camera with HD resolution
        self.f = 699.952
        self.fx = self.f
        self.fy = self.f
        self.cx = 620.197
        self.cy = 380.565
        self.k1=-0.173446
        self.k2=0.0272208

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def bbox_callback(self, msg):

        posearray = PoseArray()  # Mensagem para as coordenadas de detecções

        # Calculo da coordenada por meio da profundidade
        for box in msg.bounding_boxes:  # Para cada detecção de objetos
            x_c = int((box.xmin+box.xmax)/2)  # Extração do centro da bounding box
            y_c = int((box.ymin+box.ymax)/2)

            if len(self.depth_image) > 0:  # Se a imagem não é vazia (TODO: Melhorar essa verificação, ser mais robusta)
                depth_value = self.depth_image[y_c][x_c]  # Valor de profundidade no pixel central 
                
                if not isnan(depth_value): # Se tem um valor válido de profundidade
                    pose_box = Pose()  # Mensagem de pose individual

                    u = x_c
                    v = y_c
                    
                    # Calculo de coordenadas no sistema de coordenadas da camera
                    X_obj = depth_value*(u-self.cx)/self.fx  
                    Y_obj = depth_value*(v-self.cy)/self.fy
                    Z_obj = depth_value
                    
                    # Coordenadas para mensagem
                    pose_box.position.x = X_obj
                    pose_box.position.y = Y_obj
                    pose_box.position.z = float64(Z_obj)
                    posearray.poses.append(pose_box)

                # Timestamp da detecção para adicionar no header
                timestamp = (self.get_clock().now()).to_msg()
        
                posearray.header = msg.header                                               # assigning header of input image msg
                posearray.header.stamp = timestamp

                self.obj_pose_pub.publish(posearray)  # Publica a mensagem de detecção

        # Transformação para coordenada do mapa
        for pose in posearray.poses:
            try:
                now = rclpy.time.Time()

                # Ponto de referência - coordenada da camera
                point_camera_frame = PointStamped()
                point_camera_frame.header.frame_id = 'zedm_left_camera_optical_frame'
                point_camera_frame.point.x = 0.0
                point_camera_frame.point.y = 0.0
                point_camera_frame.point.z = 0.0

                transform = self.tf_buffer.lookup_transform('map', 'zedm_left_camera_optical_frame', now)
                point_robot_frame = tf2_geometry_msgs.do_transform_point(point_camera_frame, transform)
                self.get_logger().info(f'Camera point: {point_robot_frame.point.x}, {point_robot_frame.point.y}, {point_robot_frame.point.z}')

                # Ponto da detecção do objeto
                point_camera_frame = PointStamped()
                point_camera_frame.header.frame_id = 'zedm_left_camera_optical_frame'
                point_camera_frame.point.x = pose.position.x
                point_camera_frame.point.y = pose.position.y
                point_camera_frame.point.z = pose.position.z

                transform = self.tf_buffer.lookup_transform('map', 'zedm_left_camera_optical_frame', now)
                point_robot_frame = tf2_geometry_msgs.do_transform_point(point_camera_frame, transform)
                self.get_logger().info(f'Image point: {point_robot_frame.point.x}, {point_robot_frame.point.y}, {point_robot_frame.point.z}')

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'Could not transform point: {e}')
            
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")  # Decodificação da informação de depth
        self.depth_info = msg  # Mensagem completa com informações
        self.depth_image = depth_image  # Imagem convertida


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
