#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

void topicCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%f %f %f'", msg->r,  msg->g, msg->b);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ros_blink1_driver_node");
    auto subscription = node->create_subscription<std_msgs::msg::ColorRGBA>("blink1_color", 10, topicCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
