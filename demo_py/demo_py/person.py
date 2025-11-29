import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self,name,age):
        super().__init__('person_node')
        self.name = name
        self.age = age

    def print_info(self):
        self.get_logger().info(f"Name: {self.name}, Age: {self.age}")


def main():
    rclpy.init()
    person_node = PersonNode("wjz", 20)
    person_node.print_info()
    rclpy.spin(person_node)
    rclpy.shutdown()
