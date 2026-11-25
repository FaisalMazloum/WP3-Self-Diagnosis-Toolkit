#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


class wheel_controller : public rclcpp::Node {


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist twist_msg;
    rclcpp::TimerBase::SharedPtr timer_;

    double linear_speed;
    double angular_speed;
    int counter = 0;

    void my_callback() {
        twist_msg.angular.z = angular_speed;
        twist_msg.linear.x = linear_speed;
        publisher_->publish(twist_msg);
        counter++;
        RCLCPP_INFO_STREAM(this->get_logger(), "Published Counter: " << counter << " || Linear x: " << linear_speed << " || Angular z:" << angular_speed << std::endl);
    }



public:
    wheel_controller() : Node("wheel_controller_node") {

        this->declare_parameter("linear_speed", 5.0);
        this->declare_parameter("angular_speed", 1.0);
        linear_speed = this->get_parameter("linear_speed").as_double();
        angular_speed = this->get_parameter("angular_speed").as_double();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("my_topic", 10);
        timer_ = this->create_wall_timer(0.05s, std::bind(&wheel_controller::my_callback, this));
    }


};







int main (int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<wheel_controller>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}