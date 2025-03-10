#include "nodes/io_node.hpp"
#include "helper.hpp"

namespace nodes {
    IoNode::IoNode() : Node("ionode")  {
        // Initialize the publisher
        LED_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(Topic::set_rgb_leds, 10);

        // Init subscriber
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
           Topic::buttons, 10, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::set_rgb_leds.c_str());
        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::buttons.c_str());
    }


    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = msg->data;

        auto msgPublish = std_msgs::msg::UInt8();
        msgPublish.data = get_button_pressed();
        LED_publisher_->publish(msgPublish);
        RCLCPP_INFO(this->get_logger(), "Received: %d", msg->data);
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }
}
