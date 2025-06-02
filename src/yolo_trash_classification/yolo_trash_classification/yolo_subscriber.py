import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch
import cv2
import os
import sys

yolov5_path = '/home/jonghyeon/ros2_trash_detect/yolov5'
sys.path.insert(0, yolov5_path)

from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes
from utils.torch_utils import select_device

class YoloV5SubscriberNode(Node):
    def __init__(self, model_path, device='cpu'):
        super().__init__('yolo_v5_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.listener_callback,
            10)
        self.subscription 
        self.bridge = CvBridge()
        self.device = select_device(device)
        self.model = DetectMultiBackend(model_path, device=self.device)
        self.model.eval()
        self.labels = ['vinyl', 'pet', 'paper', 'can', 'glass']
        self.image_received = False
       
        self.timer = self.create_timer(1.0, self.timer_callback)

    def listener_callback(self, data):
        self.image_received = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = torch.from_numpy(frame_rgb).to(self.device)
        img = img.permute(2, 0, 1).float()  # (HWC to CHW)
        img /= 255.0  
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        pred = self.model(img, augment=False, visualize=False)
        pred = non_max_suppression(pred, 0.25, 0.45, None, False, max_det=1000)

        for det in pred: 
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], cv_image.shape).round()
                for *xyxy, conf, cls in det:
                    label = self.labels[int(cls)]
                    self.get_logger().info(f'Detect result: {label} / Probability: {conf:.2f}')

    def timer_callback(self):
        if not self.image_received:
            self.get_logger().warn('이미지가 들어오지 않았습니다')
        self.image_received = False  

def main(args=None):
    rclpy.init(args=args)
    model_path = '/home/jonghyeon/ros2_trash_detect/src/yolo_trash_classification/best.pt'  
    node = YoloV5SubscriberNode(model_path=model_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
