#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace nodes {
    class ImuNode : public rclcpp::Node {
    public:
        // Constructor
        ImuNode();
        // Destructor (default)
        ~ImuNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;


    private:
        // Variable to store the last received button press value
        double freq = 250; // loop time = 1/freq
        int button_pressed_ = -1;
        u_char speedLeft = 127;
        u_char speedRight = 127;
        u_char maxSpeed = 145;
        u_char noSpeed = 127;

        int stav = -1;
        int zed = -1;
        int zatacka = -1;

        u_char frontLed = 255;
        u_char backLed = 255;
        u_char leftLed = 255;
        u_char rightLed = 255;

        double front;
        double back;
        double left;
        double right;

        int i = 0;
        double gyro_calib=0;
        double gyro_z;
        double gyro_offset;
        double dt;
        double yaw;

        int32_t encoderValues[2] = {0, 0};
        std::vector<float> lidarRanges;
        int lidarMaxIndex = 1079;
        double maxWallDistance = 0.4;
        double minLidarDistance = 0.12;

        // Subscribers and publishers
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_speed_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr ultrasounds_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

        rclcpp::Time start_time_;
        rclcpp::Time last_time_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Callback - preprocess received message
        void on_button_callback(std_msgs::msg::UInt8::SharedPtr msg);
        void ultrasounds_callback(std_msgs::msg::UInt8MultiArray::SharedPtr msg);
        void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
        void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
        void timer_callback();


        double getLidarRange(double midAngle, int numOfPoints, int angle);
        double calculateRotation();
    };
}
