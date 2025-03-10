#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        // Constructor
        IoNode();
        // Destructor (default)
        ~IoNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;

    private:
        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // rclcpp::Node::SharedPtr node_;

        // Subscriber for button press messages
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr LED_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;

        // Callback - preprocess received message
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    };
}
