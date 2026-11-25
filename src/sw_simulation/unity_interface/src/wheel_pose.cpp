#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::placeholders;



class my_class : public rclcpp::Node {

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    nav_msgs::msg::Odometry odom_;

    void my_callback(const geometry_msgs::msg::Twist msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Linear Recieved: " << msg.linear.x << " || Angular Recieved: " << msg.angular.z << std::endl);
    }


public:
    my_class() : Node("my_class_node") {
        // subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("my_topic", 10, std::bind(&my_class::my_callback, this, _1));
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("my_topic", 10, std::bind(&my_class::my_callback, this, _1));
    }
};




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<my_class>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}