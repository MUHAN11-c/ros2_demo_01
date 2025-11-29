import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    # 初始化ROS2 Python客户端库
    # args参数用于传递命令行参数给ROS2系统
    rclpy.init(args=args)
    
    # 创建一个名为'python_node'的ROS2节点实例
    # 节点是ROS2中执行计算的基本单元
    node = Node('python_node')
    
    # 使用节点的日志器输出信息级别日志
    # 日志内容为'Hello ROS2!'，将显示在控制台
    node.get_logger().info('Hello ROS2!')
    
    # 进入ROS2事件循环，保持节点运行直到收到退出信号
    # 这会阻塞当前线程，等待回调和事件
    rclpy.spin(node)
    
    # 销毁节点，释放相关资源
    # 通常在程序结束前调用，确保资源正确清理
    node.destroy_node()
    
    # 关闭ROS2 Python客户端库
    # 清理所有初始化时创建的资源
    rclpy.shutdown()