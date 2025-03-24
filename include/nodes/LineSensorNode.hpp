//
// Created by petr on 3/17/25.
//

#pragma once


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace nodes {
class LineSensorNode : public rclcpp::Node {

    public:
        // Constructor
        LineSensorNode();
        // Destructor (default)
        ~LineSensorNode() override = default;

    private:
        std_msgs::msg::UInt8MultiArray msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        u_char speedLeft = 127;
        u_char speedRight = 127;
        u_char direction = 0;
        // Publishers
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_speed_publisher_;
        // Subscribers
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensor_subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoders_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;

        void on_lineSensor_callback(std_msgs::msg::UInt16MultiArray::SharedPtr msg);
        void timer_callback();

    };
}
