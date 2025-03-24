//
// Created by petr on 3/17/25.
//

#include "nodes/LineSensorNode.hpp"

#include "helper.hpp"

namespace nodes {
    LineSensorNode::LineSensorNode() : Node("ionode_lineSensor"), start_time_(this->now())  {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1)),
                std::bind(&LineSensorNode::timer_callback, this));
        // Initialize the publisher
        motor_speed_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::motor_speeds, 10);

        // Init subscribers
        line_sensor_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
           Topic::line_sensors, 10, std::bind(&LineSensorNode::on_lineSensor_callback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::line_sensors.c_str());
        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::motor_speeds.c_str());
    }

    // levy je 0, pravy je 1

    void LineSensorNode::on_lineSensor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Direction: %d \t LINE DATA \t Leva: %d \t Prava: %d",direction, msg->data[0], msg->data[1]);

        if (msg->data[0] < 500 && msg->data[1] > 500) {
            direction = 1; // right
        }
        else if (msg->data[0] > 500 && msg->data[1] < 500) {
            direction = 2; // left
        }
        else{
            direction = 0; // straight}
        }
    }
    void LineSensorNode::timer_callback() {
        switch (direction) {
            case 0:
                speedLeft = 140;
                speedRight = 140;
            break;
            case 1:
                speedRight = 127;
            break;
            case 2:
                speedLeft = 127;
            break;
        }

        RCLCPP_INFO(this->get_logger(), "Direction: %d \tSpeed \t Leva: %d \t Prava: %d",direction, speedLeft, speedRight);
        msgMotorSpeeds.data = {speedLeft, speedRight};
        motor_speed_publisher_->publish(msgMotorSpeeds);
    }
}
