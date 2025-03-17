//
// Created by petr on 3/17/25.
//

#include "nodes/LineSensorNode.hpp"

#include "helper.hpp"

namespace nodes {
    LineSensorNode::LineSensorNode() : Node("ionode_lineSensor")  {
        // Initialize the publisher

        // Init subscribers
        line_sensor_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
           Topic::line_sensors, 10, std::bind(&LineSensorNode::on_lineSensor_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::line_sensors.c_str());
    }

    // levy je 0, pravy je 1

    uint LineSensorNode::on_lineSensor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received LINE DATA \t Leva: %d \t Prava: %d", msg->data[0], msg->data[1]);

        if (msg->data[0] < 500 && msg->data[1] > 500) {
            return 1; // right
        }
        if (msg->data[0] > 500 && msg->data[1] < 500) {
            return 2; // left
        }
        /*(msg->data[0] < 500 && msg->data[1] < 500)*/
        return 0; // straight



    }
}
