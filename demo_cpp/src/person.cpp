#include "rclcpp/rclcpp.hpp"
#include <string>


class PersonNode : public rclcpp::Node 
{
private:
    std::string name_;
    int age_;

public:
    PersonNode(const std::string &node_name, const std::string &name, int age)
        : Node(node_name)
    {
        this->name_ = name;
        this->age_ = age;
    };
    void print_info()
    {
        RCLCPP_INFO(this->get_logger(), "Name: %s, Age: %d", this->name_.c_str(), this->age_);
    };
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PersonNode>("Person_cpp_node","wjz",18);
    node->print_info();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

