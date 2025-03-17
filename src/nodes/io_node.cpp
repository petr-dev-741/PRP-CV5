#include "nodes/io_node.hpp"
#include "helper.hpp"
#include <nodes/LineSensorNode.hpp>

namespace nodes {
    IoNode::IoNode() : Node("ionode")  {
        // Initialize the publisher
        LED_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(Topic::set_rgb_leds, 10);

        motor_speed_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::motor_speeds, 10);

        // Init subscribers
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
           Topic::buttons, 10, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

        encoders_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
           Topic::encoders, 10, std::bind(&IoNode::on_encoder_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::set_rgb_leds.c_str());
        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::buttons.c_str());

        auto msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        msgMotorSpeeds.data = {127,127};
        motor_speed_publisher_->publish(msgMotorSpeeds);
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {

        RCLCPP_INFO(this->get_logger(), "Received button: %d", msg->data);
    }

    void IoNode::on_encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
    // for (int i = 0; i < sizeof(msg)/sizeof(uint32_t);++ i) {
    //     RCLCPP_INFO(this->get_logger(), "Received encoder: %d", msg->data[i]);
    // }
        // RCLCPP_INFO(this->get_logger(), "Received from motor encoder: %d , %d", msg->data[0],msg->data[1]);


        auto msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        msgMotorSpeeds.data = {speedLeft,speedRight};
        motor_speed_publisher_->publish(msgMotorSpeeds);
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }
}

//button_pressed_ = msg->data;

// auto msgPublish = std_msgs::msg::UInt8();
// // msgPublish.data = {255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// LED_publisher_->publish(msgPublish);
//
// switch (get_button_pressed()) {
//     case 0:
//         if (speed < 245)
//             speed += 10;
//         break;
//     case 1:
//         speed = 127;
//         break;
//     case 2:
//         if (speed > 10)
//             speed -= 10;
//         break;