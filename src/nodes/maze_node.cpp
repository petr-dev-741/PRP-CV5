#include "nodes/maze_node.hpp"
#include "helper.hpp"
#include <math.h>

namespace nodes {
    MazeNode::MazeNode() : Node("node") {
        // Initialize the publisher
        motor_speed_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::motor_speeds, 10);

        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 10);

        // Init subscribers
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            Topic::buttons, 10, std::bind(&MazeNode::on_button_callback, this, std::placeholders::_1));

        ultrasounds_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            Topic::ultrasounds, 10, std::bind(&MazeNode::ultrasounds_callback, this, std::placeholders::_1));

        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            Topic::lidar, 10, std::bind(&MazeNode::lidar_callback, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            Topic::imu, 10, std::bind(&MazeNode::imu_callback, this, std::placeholders::_1));

        camera_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            Topic::camera, 10, std::bind(&MazeNode::camera_callback, this, std::placeholders::_1));

        // Create a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1 / freq * 1000)),
            std::bind(&MazeNode::timer_callback, this));

        auto rgb_leds = std_msgs::msg::UInt8MultiArray();
        rgb_leds.data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        led_publisher_->publish(rgb_leds);

        auto msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        msgMotorSpeeds.data = {127, 127};
        motor_speed_publisher_->publish(msgMotorSpeeds);
    }


    int MazeNode::get_button_pressed() const {
        return button_pressed_;
    }


    //choose number of points for calculation or angle to measure - angle has higher priority
    double MazeNode::getLidarRange(const double midAngle, int numOfPoints = 1, const int angle = 0) {
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

    double MazeNode::calculateRotation() {
        double beta = 10;
        double betaRad = beta / 180 * M_PI;
        int points = 5;

        double angle_left = 0;
        double angle_right = 0;

        bool valid_r = true;
        bool valid_l = true;

        double left_b = getLidarRange(90 + beta, points, 0);
        double left_m = getLidarRange(90, points, 0);
        double left_f = getLidarRange(90 - beta, points, 0);

        double right_b = getLidarRange(270 - beta, points, 0);
        double right_m = getLidarRange(270, points, 0);
        double right_f = getLidarRange(270 + beta, points, 0);

        if (right_b < 0 or right_m < 0 or right_f < 0)
            valid_r = false;

        if (left_b < 0 or left_m < 0 or left_f < 0)
            valid_l = false;

        if (valid_l) {
            double y_fml = sqrt(left_m * left_m + left_f * left_f - 2 * left_f * left_m * cos(betaRad));
            double y_bml = sqrt(left_m * left_m + left_b * left_b - 2 * left_b * left_m * cos(betaRad));

            double angle_lf = asin(left_f / y_fml * sin(betaRad)) * 180 / M_PI - 90;
            double angle_lb = 90 - asin(left_b / y_bml * sin(betaRad)) * 180 / M_PI;

            // RCLCPP_INFO(this->get_logger(), "front %f,  middle %f, back %f, front_dist %f, back_dist %f", left_f, left_m, left_b, y_fm, y_bm);
            //RCLCPP_INFO(this->get_logger(), "lf %f,  lb %f", angle_lf, angle_lb);

            if (angle_lb + angle_lf > 3)
                valid_l = false;

            angle_left = pow(-1, y_fml <
                                 y_bml) * (angle_lb - angle_lf) / 2;
        }

        if (valid_r) {
            double y_fmr = sqrt(right_m * right_m + right_f * right_f - 2 * right_f * right_m * cos(betaRad));
            double y_bmr = sqrt(right_m * right_m + right_b * right_b - 2 * right_b * right_m * cos(betaRad));

            double angle_rf = asin(right_f / y_fmr * sin(betaRad)) * 180 / M_PI - 90;
            double angle_rb = 90 - asin(right_b / y_bmr * sin(betaRad)) * 180 / M_PI;

            // RCLCPP_INFO(this->get_logger(), "front %f,  middle %f, back %f, front_dist %f, back_dist %f", right_f, right_m, right_b, y_fm, y_bm);
            //RCLCPP_INFO(this->get_logger(), "rf %f,  rb %f", angle_rf, angle_rb);

            if (abs(angle_rb + angle_rf) > 3)
                valid_r = false;

            angle_right = pow(-1, y_fmr > y_bmr) * (angle_rb - angle_rf) / 2;
        }
        //RCLCPP_INFO(this->get_logger(), "left %f,  right %f, final %f", angle_left, angle_right, (angle_left+angle_right)/2);

        if (valid_l and valid_r)
            return (angle_left + angle_right) / 2;

        if (valid_l)
            return angle_left;

        if (valid_r)
            return angle_right;

        return 0;
    }

    void MazeNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
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

    void MazeNode::ultrasounds_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        /*if (msg->data[1] < 15)
            zed = 1;
        else zed = 0;*/
        ultrasound = msg->data[1];
        //RCLCPP_INFO(this->get_logger(), "Left: %d, Straight: %d, Right: %d", msg->data[0], msg->data[1], msg->data[2]);
    }

    void MazeNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        lidarRanges = msg->ranges;
        front = getLidarRange(0, 20, 0);
        left = getLidarRange(90, 20, 0);
        back = getLidarRange(180, 20, 0);
        right = getLidarRange(270, 20, 0);

        if (front < 0.20 or ultrasound < 20) {
            zed = 1;
        } else zed = 0;

        if (left >= 0.4)
            zatacka = 1;
        if (right >= 0.4)
            zatacka = 2;
        if (left <= 0.4 and right <= 0.4 and front <= 0.25) {
            zatacka = 3;
            zed = 1;
        }
        if ((left >= 0.4 and right >= 0.4 and back >= 0.4) or (left >= 0.4 and front >= 0.4 and back >= 0.4) or (
                front >= 0.4 and right >= 0.4 and back >= 0.4) or (
                left >= 0.4 and right >= 0.4 and front >= 0.4)) {
            zatacka = 0; // krizovatka
        }


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

    void MazeNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        gyro_z = msg->angular_velocity.z;
        //RCLCPP_INFO(this->get_logger(), "Gyro_z: %f", gyro_z);
    }

    void MazeNode::camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        const std::vector<uint8_t> &compressed_data = msg->data;
        cv::Mat compressedMat(compressed_data, true);
        cv::Mat image = cv::imdecode(compressedMat, cv::IMREAD_COLOR);
        arucos = detektor.detect(image);
        if (!arucos.empty()) {
            if (arucos[0].id >= 10) {
                krizovatka = arucos[0].id;
                if (arucos[0].id < 10 and krizovatka == -1) {
                    krizovatka = arucos[0].id;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Detekovane id: %d", arucos[0].id);
        }
    }

    void MazeNode::timer_callback() {
        auto msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        if (button_pressed_ == 1) {
            dt = (this->now() - last_time_).seconds();
            last_time_ = this->now();
            yaw += (gyro_z - gyro_offset) * dt * 180 / M_PI;
        }


        switch (stav) {
            case 0: // Kalibrace
                gyro_calib += gyro_z;
                i += 1;
                if (this->now() - start_time_ > std::chrono::seconds(5)) {
                    gyro_offset = gyro_calib / i;
                    yaw = -calculateRotation();
                    RCLCPP_INFO(this->get_logger(), "Calibration done, gyro_offset: %f, samples: %d", gyro_offset, i);
                    button_pressed_ = 1;
                    stav = 3;
                }
                break;
            case 1:
                //if (abs(yaw) > 7) stav = 3;
                if (left < 0.5 and right < 0.5) {
                    speedRight = maxSpeed - (maxSpeed - noSpeed) * (-regulator_prava_stena.step(0.2 - right, dt));
                    speedLeft = maxSpeed - (maxSpeed - noSpeed) * (-regulator_leva_stena.step(0.2 - left, dt));
                    //RCLCPP_INFO(this->get_logger(), "Mezi zdmi, L: %f, R: %f", -regulator_leva_stena.step(0.2 - left, dt), -regulator_prava_stena.step(0.2 - right, dt));
                } else if ((right != 0.2) and right < 0.5) {
                    speedLeft = maxSpeed - (maxSpeed - noSpeed) * (regulator_prava_stena.step(0.2 - right, dt));
                    speedRight = maxSpeed - (maxSpeed - noSpeed) * (-regulator_prava_stena.step(0.2 - right, dt));
                    //RCLCPP_INFO(this->get_logger(), "podle prave, L: %f, R: %f", regulator_prava_stena.step(0.2 - right, dt), -regulator_prava_stena.step(0.2 - right, dt));
                } else if ((left != 0.2) and left < 0.5) {
                    speedLeft = maxSpeed - (maxSpeed - noSpeed) * (-regulator_leva_stena.step(0.2 - left, dt));
                    speedRight = maxSpeed - (maxSpeed - noSpeed) * (regulator_leva_stena.step(0.2 - left, dt));
                    //RCLCPP_INFO(this->get_logger(), "podle leve, L: %f, R: %f", -regulator_leva_stena.step(0.2 - left, dt), regulator_leva_stena.step(0.2 - left, dt));
                } else {
                    speedLeft = maxSpeed;
                    speedRight = maxSpeed;
                    //RCLCPP_INFO(this->get_logger(), "neni zed, L: %d, R: %d", speedLeft, speedRight);
                }

                speedLeft = speedLeft + std::copysign(1, yaw);
                speedRight = speedRight - std::copysign(1, yaw);
                //RCLCPP_INFO(this->get_logger(), "L: %d, R: %d", speedLeft, speedRight);


                //speedLeft = maxSpeed + 5*(zedRight - zedLeft)/255;
                //speedRight = maxSpeed + 5*(zedLeft - zedRight)/255;
                if (zed == 1) {
                    stav = 2;
                }
                //RCLCPP_INFO(this->get_logger(), "JEDU %f", regulator_prava_stena.step(0.2 - right, dt));
                msgMotorSpeeds.data = {speedLeft, speedRight};
                motor_speed_publisher_->publish(msgMotorSpeeds);
                break;
            case 2:
                if (zatacka == 0) {
                    zatacka = krizovatka % 10;
                    krizovatka = -1;
                }
                if (zatacka == 0) {
                    stav = 1;
                }
                if (zatacka == 1) {
                    yaw -= 90;
                    stav = 3;
                }
                if (zatacka == 2) {
                    yaw += 90;
                    stav = 3;
                }
                if (zatacka == 3) {
                    yaw += 180;
                    stav = 3;
                }
                break;
            case 3:
                regulator_otaceni.step(yaw, dt);
                speedLeft = 127 + round(regulator_otaceni.step(yaw, dt)) + std::copysign(2, yaw);
                speedRight = 127 - round(regulator_otaceni.step(yaw, dt)) - std::copysign(2, yaw);
                msgMotorSpeeds.data = {speedLeft, speedRight};
                motor_speed_publisher_->publish(msgMotorSpeeds);
                RCLCPP_INFO(this->get_logger(), "ZATACIM yaw: %f,reg: %f", yaw,
                            ceil(regulator_otaceni.step(yaw, dt))*3);
                if (abs(yaw) < 0.2) {
                    if (calculateRotation() == 0)
                        yaw = -calculateRotation();
                    stav = 1;
                }
                break;
        }
        //RCLCPP_INFO(this->get_logger(), "front: %f, back: %f, left: %f, right: %f", front, back, left, right);
        RCLCPP_INFO(this->get_logger(), "zatacka %d", zatacka);
    }
}
