#include "nodes/imu_node.hpp"
#include "helper.hpp"
#include <math.h>

namespace nodes {
    ImuNode::ImuNode() : Node("node") {
        // Initialize the publisher
        motor_speed_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::motor_speeds, 10);

        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 10);

        // Init subscribers
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            Topic::buttons, 10, std::bind(&ImuNode::on_button_callback, this, std::placeholders::_1));

        ultrasounds_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            Topic::ultrasounds, 10, std::bind(&ImuNode::ultrasounds_callback, this, std::placeholders::_1));

        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            Topic::lidar, 10, std::bind(&ImuNode::lidar_callback, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            Topic::imu, 10, std::bind(&ImuNode::imu_callback, this, std::placeholders::_1));

        // Create a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1 / freq * 1000)),
            std::bind(&ImuNode::timer_callback, this));

        auto rgb_leds = std_msgs::msg::UInt8MultiArray();
        rgb_leds.data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        led_publisher_->publish(rgb_leds);

        auto msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        msgMotorSpeeds.data = {127, 127};
        motor_speed_publisher_->publish(msgMotorSpeeds);
    }

    void ImuNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = msg->data;
        start_time_ = this->now();
        last_time_ = this->now();
        if (msg->data == 0) {
            stav = 0; // Kalibrace
            RCLCPP_INFO(this->get_logger(), "Calibration in process...");
        }
        if (msg->data == 1) {
            stav = 1; // Jizda
        }
        if (msg->data == 2) {
            stav = -1; // Stop
        }
        RCLCPP_INFO(this->get_logger(), "Received button: %d", msg->data);
    }

    void ImuNode::ultrasounds_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data[1] < 15)
            zed = 1;
        else zed = 0;
        //RCLCPP_INFO(this->get_logger(), "Left: %d, Straight: %d, Right: %d", msg->data[0], msg->data[1], msg->data[2]);
    }

    void ImuNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        lidarRanges = msg->ranges;
        front = getLidarRange(0, 20, 0);
        left = getLidarRange(90, 20, 0);
        back = getLidarRange(180, 20, 0);
        right = getLidarRange(270, 20, 0);
        if (left == 1)
            zatacka = 0;
        if (right == 1)
            zatacka = 1;

        auto rgb_leds = std_msgs::msg::UInt8MultiArray();
        // zadni, prava, leva, predni
        if (back < 0.22)
            backLed = 255;
        else backLed = 0;
        if (right < 0.22)
            rightLed = 255;
        else rightLed = 0;
        if (left < 0.22)
            leftLed = 255;
        else leftLed = 0;
        if (front < 0.22)
            frontLed = 255;
        else frontLed = 0;
        rgb_leds.data = {0, backLed, 0, 0, rightLed, 0, 0, leftLed, 0, 0, frontLed, 0};
        led_publisher_->publish(rgb_leds);
    }

    //choose number of points for calculation or angle to measure - angle has higher priority
    double ImuNode::getLidarRange(const double midAngle, int numOfPoints = 1, const int angle = 0) {
        double distance = 0.0;
        if (angle > 0 && angle < 360)
            numOfPoints = angle * 3; // 1080/360
        double sum = 0;
        double ranges[numOfPoints];
        for (int i = 0; i < numOfPoints; i++) {
            int index = static_cast<int>(round(midAngle * 3.0) - floor(numOfPoints / 2.0)) + i;
            if (index < 0)
                index = lidarMaxIndex + index;
            else if (index > lidarMaxIndex)
                index = index - lidarMaxIndex;
            ranges[i] = lidarRanges[index];
        }
        std::sort(ranges, ranges + numOfPoints);
        if (numOfPoints % 2 != 0) // liche cislo
            distance = ranges[(numOfPoints + 1) / 2];
        else // sude
            distance = (ranges[numOfPoints / 2] + ranges[numOfPoints / 2 + 1]) / 2.0;

        // for (int i = 0; i < numOfPoints; i++) {
        // printf("%f ", ranges[i]);
        // }
        // printf("\n");

        if (distance > maxWallDistance)
            distance = 1;
        else if (distance < minLidarDistance)
            distance = 0;


        return distance;
    }

    double ImuNode::calculateRotation() {
        double beta = 15;
        double betaRad = beta / 180 * M_PI;
        int points = 5;

        double left_b = getLidarRange(90 + beta, points, 0);
        double left_m = getLidarRange(90, points, 0);
        double left_f = getLidarRange(90 - beta, points, 0);

        double right_b = getLidarRange(270 + beta, points, 0);
        double right_m = getLidarRange(270, points, 0);
        double right_f = getLidarRange(270 - beta, points, 0);

        double y_fm = sqrt(left_m * left_m + left_f * left_f - 2 * left_f * left_m * cos(betaRad));
        double y_bm = sqrt(left_m * left_m + left_b * left_b - 2 * left_b * left_m * cos(betaRad));

        double angle_lf = asin(left_f / y_fm * sin(betaRad)) * 180 / M_PI - 90;
        double angle_lb = 90 - asin(left_b / y_bm * sin(betaRad)) * 180 / M_PI;

        return angle_lf;
        RCLCPP_INFO(this->get_logger(), "front %f,  back %f", angle_lf, angle_lb);
    }

    void ImuNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        gyro_z = msg->angular_velocity.z;
        //RCLCPP_INFO(this->get_logger(), "Gyro_z: %f", gyro_z);
    }

    void ImuNode::timer_callback() {
        auto msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        if (button_pressed_ == 1) {
            dt = (this->now() - last_time_).seconds();
            last_time_ = this->now();
            yaw += ((gyro_z - gyro_offset) * dt) * 180 / M_PI;
        }
        switch (stav) {
            case 0: // Kalibrace
                gyro_calib += gyro_z;
                i += 1;
                if (this->now() - start_time_ > std::chrono::seconds(5)) {
                    gyro_offset = gyro_calib / i;
                    yaw = 0;
                    RCLCPP_INFO(this->get_logger(), "Calibration done, gyro_offset: %f, samples: %d", gyro_offset, i);
                    button_pressed_ = 1;
                    stav = 1;
                }
                break;
            case 1:
                if (yaw > 7) {
                    stav = 3;
                }
                speedLeft = maxSpeed;
                speedRight = maxSpeed;
                if (right < 1) {
                    if (right < 0.15) {
                        speedLeft = noSpeed + (maxSpeed - noSpeed) / 2;
                        speedRight = maxSpeed;
                    }
                    if (right > 0.25) {
                        speedLeft = maxSpeed;
                        speedRight = noSpeed + (maxSpeed - noSpeed) / 2;
                    }
                } else if (right >= 1 and left < 1) {
                    if (left < 0.15) {
                        speedRight = noSpeed + (maxSpeed - noSpeed) / 2;
                        speedLeft = maxSpeed;
                    }
                    if (left > 0.25) {
                        speedRight = maxSpeed;
                        speedLeft = noSpeed + (maxSpeed - noSpeed) / 2;
                    }
                }

            //speedLeft = maxSpeed + 5*(zedRight - zedLeft)/255;
            //speedRight = maxSpeed + 5*(zedLeft - zedRight)/255;
                if (zed == 1) {
                    stav = 2;
                }
                RCLCPP_INFO(this->get_logger(), "JEDU");
                msgMotorSpeeds.data = {speedLeft, speedRight};
                motor_speed_publisher_->publish(msgMotorSpeeds);
                break;
            case 2:
                if (zatacka == 1) {
                    yaw += 90;
                    stav = 3;
                }
                if (zatacka == 0) {
                    yaw -= 90;
                    stav = 3;
                }
                break;
            case 3:
                speedLeft = 127 + (133 - 127) * std::copysign(1, yaw);
                speedRight = 127 + (133 - 127) * std::copysign(1, -yaw);
                msgMotorSpeeds.data = {speedLeft, speedRight};
                motor_speed_publisher_->publish(msgMotorSpeeds);
                RCLCPP_INFO(this->get_logger(), "ZATACIM yaw: %f", yaw);
                if (abs(yaw) < 0.1) {
                    stav = 1;
                }
                break;
        }
    }

    int ImuNode::get_button_pressed() const {
        return button_pressed_;
    }
}
