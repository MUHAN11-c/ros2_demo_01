#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "custom_interfaces/srv/patrol.hpp"
#include <chrono>
using namespace std::chrono_literals;
using Patrol = custom_interfaces::srv::Patrol;

class PatrolClientNode : public rclcpp::Node
{
private:
    rclcpp::Client<Patrol>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double target_x_{};
    double target_y_{};

public:
    PatrolClientNode() : Node("demo_cpp_client_node")
    {
        client_ = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(1s, std::bind(&PatrolClientNode::timer_callback, this));
        srand(time(NULL));
    }
    void timer_callback()
    {
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        auto request = std::make_shared<Patrol::Request>();
        request->target_x = (float)(rand() % 10);
        request->target_y = (float)(rand() % 10);
        target_x_ = request->target_x;
        target_y_ = request->target_y;
        RCLCPP_INFO(this->get_logger(), "Sending request: target_x: %f, target_y: %f", request->target_x, request->target_y);

        auto result_future = client_->async_send_request(request, std::bind(&PatrolClientNode::handle_response, this, std::placeholders::_1));
    }
    void handle_response(rclcpp::Client<Patrol>::SharedFuture future)
    {
        auto response = future.get();
        if (response->result == Patrol::Response::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "服务响应: 成功移动到 (%.2f, %.2f)",
                        target_x_, target_y_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "服务响应:  失败");
        }
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}