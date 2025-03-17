//
// Created by petr on 3/17/25.
//

#pragma once


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

namespace nodes {
class LineSensorNode : public rclcpp::Node {

    public:
        // Constructor
        LineSensorNode();
        // Destructor (default)
        ~LineSensorNode() override = default;

        uint on_lineSensor_callback(std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    private:
        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensor_subscription_;


    };
}
