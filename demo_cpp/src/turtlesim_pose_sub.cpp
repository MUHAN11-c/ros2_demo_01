#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

class turtlesim_pose_sub : public rclcpp::Node
{
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};
    double max_linear_vel_{3.0};

public:
    turtlesim_pose_sub()
        : Node("turtlesim_pose_sub_node")
    {
        this->subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&turtlesim_pose_sub::pose_callback, this, std::placeholders::_1));
        this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    };

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        double current_x = pose->x;
        double current_y = pose->y;
        RCLCPP_INFO(this->get_logger(), "Current position: x=%f, y=%f", current_x, current_y);

        double distance = std::sqrt(std::pow(current_x - target_x_, 2) + std::pow(current_y - target_y_, 2));
        double angle = std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

        if(distance>0.1)
        {
            if(fabs(angle)>0.1)
            {
                twist_msg.angular.z = k_ * angle;
            }
            else
            {
                twist_msg.linear.x = k_ * distance;
                if(twist_msg.linear.x>max_linear_vel_)
                {
                    twist_msg.linear.x = max_linear_vel_;
                }
            }
        }
        this->publisher_->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Pose: x=%f, y=%f, theta=%f", pose->x, pose->y, pose->theta);
    };
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtlesim_pose_sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
