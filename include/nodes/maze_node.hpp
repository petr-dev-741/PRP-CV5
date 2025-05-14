#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "algorithms/pid.hpp"
#include "algorithms/aruco_detector.hpp"

namespace nodes {
    class MazeNode : public rclcpp::Node {
    public:
        // Constructor
        MazeNode();
        // Destructor (default)
        ~MazeNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;


    private:
        // Variable to store the last received button press value
        double freq = 100; // loop time = 1/freq
        int button_pressed_ = -1;
        u_char speedLeft = 127;
        u_char speedRight = 127;
        u_char maxSpeed = 145;
        u_char noSpeed = 127;

        int stav = -1;
        int zed = -1;
        int zatacka = -1;
        int krizovatka = -1;
        int stred_zatacky = 0;

        u_char frontLed = 255;
        u_char backLed = 255;
        u_char leftLed = 255;
        u_char rightLed = 255;

        double ultrasound;
        double front =1;
        double back=1;
        double left=1;
        double right=1;

        int i = 0;
        double gyro_calib=0;
        double gyro_z;
        double gyro_offset;
        double dt;
        double yaw;

        std::vector<float> lidarRanges;
        int lidarMaxIndex = 1079;
        double maxWallDistance = 0.7;
        double minLidarDistance = 0.12;

        algorithms::Pid regulator_prava_stena = algorithms::Pid(2.5,0,0.2);
        algorithms::Pid regulator_leva_stena = algorithms::Pid(2.5,0,0.2);
        algorithms::Pid regulator_otaceni = algorithms::Pid(0.1,0,0.2);

        algorithms::ArucoDetector detektor = algorithms::ArucoDetector();
        std::vector<algorithms::ArucoDetector::Aruco> arucos;

        // Subscribers and publishers
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_speed_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr ultrasounds_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber_;

        rclcpp::Time start_time_;
        rclcpp::Time last_time_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Callback - preprocess received message
        void on_button_callback(std_msgs::msg::UInt8::SharedPtr msg);
        void ultrasounds_callback(std_msgs::msg::UInt8MultiArray::SharedPtr msg);
        void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
        void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
        void camera_callback(sensor_msgs::msg::CompressedImage::SharedPtr msg);
        void timer_callback();


        double getLidarRange(double midAngle, int numOfPoints, int angle);
        double calculateRotation();
    };
}
