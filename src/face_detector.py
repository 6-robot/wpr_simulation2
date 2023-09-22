#!/usr/bin/env python3

import rclpy
import sys
import cv2
import face_recognition
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        self.pub = self.create_publisher(RegionOfInterest, '/face_position', 10)
        self.sub = self.create_subscription(Image, "/face_detector_input", self.detect_faces_callback, 10)
        self.bridge = CvBridge()

    # 视频流处理回调函数
    def detect_faces_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # 将OpenCV的BGR格式 转换到 face_recognition支持的RGB格式
        rgb_frame = frame[:, :, ::-1]
        
        face_locations = []
        # 在视频帧里检测人脸位置
        face_locations = face_recognition.face_locations(rgb_frame)
        num_faces = len(face_locations)
        # self.get_logger().info("发现{}个人脸".format(num_faces))

        # 将人脸检测结果在原图框出来
        for (top, right, bottom, left) in face_locations:
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
            roi_msg = RegionOfInterest()
            roi_msg.x_offset = left
            roi_msg.y_offset = top
            roi_msg.width = right - left
            roi_msg.height = bottom - top
            self.pub.publish(roi_msg)

        # 显示结果图
        # cv2.imshow('Faces', frame)
        # cv2.waitKey(1)

      
def main(args):
    rclpy.init(args=args)
    face_detector_object = FaceDetector()
    face_detector_object.get_logger().warn("准备识别人脸")

    try:
        rclpy.spin(face_detector_object)
    except KeyboardInterrupt as e:
        print(e)

    face_detector_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)