#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TuetlesimNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    TuetlesimNode()
        : Node("turtlesim_pose_cpptopic_node")
    {
        this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        this->timer_ = this->create_wall_timer(1s, std::bind(&TuetlesimNode::publish_twist, this));
    };
    void publish_twist()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 1.0;
        twist_msg.angular.z = 1.0;
        this->publisher_->publish(twist_msg);
    };
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TuetlesimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
