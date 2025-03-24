#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

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
        double speedLeft = 127;
        double speedRight = 127;
        u_int16_t lineLeft = 0;
        u_int16_t lineRight = 0;
        double freq = 200; // loop time = 1/freq
        double maxSpeed = 137;
        uint16_t lineDetectionOffset = 600;
        double lineSensorLimits [4] = {19, 1020, 24, 1023};

        // rclcpp::Node::SharedPtr node_;

        // Subscriber for button press messages
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr LED_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_speed_publisher_;

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoders_subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensor_subscription_;

        rclcpp::Time start_time_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Callback - preprocess received message
        void on_button_callback(std_msgs::msg::UInt8::SharedPtr msg);

        void on_encoder_callback(std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void on_lineSensor_callback(std_msgs::msg::UInt16MultiArray::SharedPtr msg);

        void timer_callback();
    };
}
