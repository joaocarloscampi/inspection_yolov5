#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
#import time

#import argparse
import os
import sys
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())
print(os.environ['HOME'])
sys.path.append(os.environ['HOME'] + "/yolo_ws/install/inspection_yolov5/share/inspection_yolov5/scripts")

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams, letterbox
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, scale_polys, strip_optimizer, xyxy2xywh, apply_classifier)
                           #increment_path, non_max_suppression, non_max_suppression_obb, print_args, scale_coords, scale_polys, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box, plot_one_box
from utils.torch_utils import select_device, time_sync
from utils.rboxs_utils import poly2rbox, rbox2poly

from boundingboxes.msg import BoundingBox, BoundingBoxes

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        weights=os.environ['HOME'] + "/yolo_ws/install/inspection_yolov5/share/inspection_yolov5/scripts/extintor.pt"
        self.imgsz=[640, 640]  # inference size (pixels)
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.save_conf=False,  # save confidences in --save-txt labels
        self.classes=None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.visualize=False  # visualize features
        self.line_thickness=3  # bounding box thickness (pixels)
        self.hide_labels=False  # hide labels
        self.hide_conf=False  # hide confidences
        self.half=False  # use FP16 half-precision inference
        self.stride = 32
        device_num=0  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.dnn = False
        self.half=False  # use FP16 half-precision inference
        self.augment=False  # augmented inferenc
        self.s = str()
        self.flag_start = False

        # ROS parameters
        self.declare_parameter('camera_topic', '/zedm/zed_node/left/image_rect_color')
        self.declare_parameter('bounding_box_topic', 'yolov5_ros2/bounding_boxes')
        self.declare_parameter('visualization_detection', True)
        self.declare_parameter('publish_detection_bb', False)
        self.declare_parameter('publish_detection_bb_topic', 'yolov5_ros2/image_detection')

        # Initialize
        self.device = select_device(device_num)

        # Load model
        self.model = DetectMultiBackend(weights, device=self.device, dnn=self.dnn)
        stride, self.names, pt, jit, onnx, engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        imgsz = check_img_size(self.imgsz, s=stride)  # check image size

        # Half
        self.half &= (pt or jit or engine) and self.device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
        if pt or jit:
            self.model.model.half() if self.half else self.model.model.float()

        # Run inference
        self.model.warmup(imgsz=(1, 3, *imgsz), half=self.half)  # warmup

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning

        bounding_box_topic = self.get_parameter('bounding_box_topic').get_parameter_value().string_value
        self.bboxes_pub = self.create_publisher(BoundingBoxes,bounding_box_topic, 10)

        bounding_box_topic = self.get_parameter('publish_detection_bb_topic').get_parameter_value().string_value
        self.publish_detection_bb = self.get_parameter('publish_detection_bb').get_parameter_value().bool_value
        if self.publish_detection_bb:
            self.image_detect_pub = self.create_publisher(Image,bounding_box_topic, 10)

        self.visualization_detection = self.get_parameter('visualization_detection').get_parameter_value().bool_value

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        img0 = img.copy()

        '''
        # Letterbox
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]        

        # Stack
        img = np.stack(img, 0)

        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)
        #'''

        #'''
        self.img_size = 640
        img = letterbox(img, self.img_size, stride=self.stride)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(img)

        img = torch.from_numpy(img.copy()).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim

        # Inference
        visualize = False
        pred = self.model(img, augment=self.augment, visualize=visualize)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, multi_label=True, max_det=self.max_det)
        #'''

        # BoundingBoxes msg
        bboxes = BoundingBoxes()
        
        '''
        # Process detections
        for i, det in enumerate(pred):  # detections per image
            pred_poly = rbox2poly(det[:, :5]) # (n, [x1 y1 x2 y2 x3 y3 x4 y4])

            s = f'{i}: '
            s += '%gx%g ' % img.shape[2:]  # print string

            annotator = Annotator(img0, line_width=self.line_thickness, example=str(self.names))
            if len(det):

                # Rescale polys from img_size to im0 size
                pred_poly = scale_polys(img.shape[2:], pred_poly, img0.shape)
                det = torch.cat((pred_poly, det[:, -2:]), dim=1) # (n, [poly conf cls])

                # Print results
                #for c in det[:, -1].unique():
                    #n = (det[:, -1] == c).sum()  # detections per class
                    #s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                
                for *poly, conf, cls in reversed(det):
                    #line = (cls, *poly, conf) if self.save_conf else (cls, *poly)  # label format
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    annotator.poly_label(poly, label, color=colors(c, True))
        #'''
        
        # Process detections
        for i, det in enumerate(pred):                                         # detections per image
            self.im0s = img0
            #self.img = img
            #s, im0 = '', self.im0s.copy()
            im0 = self.im0s
            #frame = getattr(dataset, 'frame', 0)
            #s += '%gx%g ' % self.img.shape[2:]                                      # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]                              # normalization gain whwh
            
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    #s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "               # add to string
                    
                for *xyxy, conf, cls in reversed(det):
                    
                    # Add bbox to image
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    plot_one_box(xyxy, im0, label=label, color=colors(c, True), line_thickness=self.line_thickness-1)    

                    # Single BoundingBox msg
                    single_bbox = BoundingBox()
                    single_bbox.xmin = int(xyxy[0].item())
                    single_bbox.ymin = int(xyxy[1].item())
                    single_bbox.xmax = int(xyxy[2].item())
                    single_bbox.ymax = int(xyxy[3].item())
                    single_bbox.probability = conf.item()
                    single_bbox.id = c
                    single_bbox.class_id = self.names[c]
                    
                    bboxes.bounding_boxes.append(single_bbox)
        #'''
        timestamp = (self.get_clock().now()).to_msg()
        
        bboxes.header = data.header                                               # assigning header of input image msg
        bboxes.header.stamp = timestamp
        
        self.bboxes_pub.publish(bboxes)
        
        if not self.flag_start:
            self.flag_start = True
            self.get_logger().info('Detecção de objetos iniciada')

        if self.publish_detection_bb:
            image_message = bridge.cv2_to_imgmsg(img0, encoding="passthrough")
            self.image_detect_pub.publish(image_message)
        
        if self.visualization_detection:
            cv2.imshow("IMAGE", img0)
            cv2.waitKey(4)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

