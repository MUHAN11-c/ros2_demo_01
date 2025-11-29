import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class FaceDetectService(Node):
    def __init__(self):
        super().__init__('face_detect_service')
        self.srv = self.create_service(FaceDetector, 'face_detect', self.face_detect_callback)
        self.pub = self.create_publisher(Image, 'face_detect_response_Image', 10)
        self.bridge = CvBridge()
        self.defaut_image_path = get_package_share_directory('demo_py') + '/resource/default.jpg'
        self.test_image_path = get_package_share_directory('demo_py') + '/resource/test1.jpg'
        self.upsample_times = 1
        self.model = 'hog'

    def face_detect_callback(self,requst,response):
        if requst.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(requst.image)
        else:
            cv_image = cv2.imread(self.defaut_image_path)
        start_time = time.time()
        self.get_logger().info("加载图片完成")
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.upsample_times,model=self.model)
        end_time = time.time()
        self.get_logger().info("检测完成，耗时：{}秒".format(end_time-start_time))
        self.get_logger().info("检测到{}个人脸".format(len(face_locations)))
        response.face_num = len(face_locations)
        response.duration = end_time-start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
            cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 0), 2)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image)
        self.pub.publish(ros_image)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


