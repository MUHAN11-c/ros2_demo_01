import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class FaceDetectClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.bridge = CvBridge()
        self.defaut_image_path = get_package_share_directory('demo_py') + '/resource/test1.jpg'
        self.image = cv2.imread(self.defaut_image_path)

    def send_requst(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.get_logger().info(f"图中人脸数: {response.face_num}，耗时: {response.duration}")
        self.show_face_locations(response)
    def show_face_locations(self,response):
        for i in range(response.face_num):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.imshow("Face Locations", self.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()



def main():
    rclpy.init()
    face_detect_client = FaceDetectClient()
    face_detect_client.send_requst()
    rclpy.shutdown()