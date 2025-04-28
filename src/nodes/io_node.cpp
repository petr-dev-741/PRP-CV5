#include "nodes/io_node.hpp"
#include "helper.hpp"
#include <nodes/LineSensorNode.hpp>
#include <math.h>

namespace nodes {
    IoNode::IoNode() : Node("ionode")  {
        // Initialize the publishers
        LED_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(Topic::set_rgb_leds, 10);

        motor_speed_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::motor_speeds, 10);

        // Init subscribers
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
           Topic::buttons, 10, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

        encoders_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
           Topic::encoders, 10, std::bind(&IoNode::on_encoder_callback, this, std::placeholders::_1));

        line_sensor_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            Topic::line_sensors, 10, std::bind(&IoNode::on_lineSensor_callback, this, std::placeholders::_1));

        lidar_sensor_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            Topic::lidar, 10, std::bind(&IoNode::on_lidar_callback, this, std::placeholders::_1));

        // timer setup
        start_time_ = this->now();
        // Create a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1/freq * 1000)),
            std::bind(&IoNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::line_sensors.c_str());
        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::set_rgb_leds.c_str());
        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::buttons.c_str());
        RCLCPP_INFO(this->get_logger(), "Node setup complete for topic: %s", Topic::lidar.c_str());

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
        encoderValues[0] = msg->data[0];
        encoderValues[1] = msg->data[1];
    }

    // #################################### MAIN LOOP ####################################
    RobotPose pose = RobotPose();
    void IoNode::timer_callback() {
        double uptime = (this->now() - start_time_).seconds();

        if (uptime < 1) {
            RCLCPP_INFO(this->get_logger(), "Uptime: %f", uptime);
            RCLCPP_INFO(this->get_logger(), "Pose is: %f \t %f \t %f", pose.pos_x, pose.pos_y, pose.angle);
            return;
        }
        //RCLCPP_INFO(this->get_logger(), "Timer triggered. Publishing uptime...");

        // only P controller

        //RCLCPP_INFO(this->get_logger(), "Time is: %f", uptime);
        double lineLeftCal = (lineLeft - lineSensorLimits[0]) / (lineSensorLimits[1] - lineSensorLimits[0]);
        double lineRightCal = (lineRight - lineSensorLimits[2]) / (lineSensorLimits[3] - lineSensorLimits[2]);
        speedLeft = maxSpeed - (maxSpeed-127)  * lineLeftCal;
        speedRight = maxSpeed - (maxSpeed-127) *  lineRightCal;
        if (speedLeft < 127) speedLeft = 127;
        if (speedRight < 127) speedRight = 127;
        if (lineLeftCal > 400 and lineRightCal > 400) {
            speedLeft = maxSpeed;
            speedRight = maxSpeed;
        }

        pose = calculatePositionAndRotation(encoderValues);
        RCLCPP_INFO(this->get_logger(), "Pose is: %f \t %f \t %f", pose.pos_x, pose.pos_y, pose.angle);

        double angle = calculateRotation();
        RCLCPP_INFO(this->get_logger(), "angle %f", angle);

        // speedLeft = 130;
        // speedRight = 130;

        //RCLCPP_INFO(this->get_logger(), "Line sensors are: %hu \t %hu", lineLeft, lineRight);

        // auto msgMotorSpeeds = std_msgs::msg::UInt8MultiArray();
        // msgMotorSpeeds.data = {static_cast<uint8_t>(speedLeft),static_cast<uint8_t>(speedRight)};
        // motor_speed_publisher_->publish(msgMotorSpeeds);
        //RCLCPP_INFO(this->get_logger(), "Speeds are: %f \t %f", speedLeft, speedRight);

    }
    void IoNode::on_lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        lidarRanges = msg->ranges;
        // printf("newMeasure\n");
        // double front = getLidarRange(0, 20, 0);
        // double left = getLidarRange(90, 20, 0);
        // double back = getLidarRange(180, 20, 0);
        // double right = getLidarRange(270, 20, 0);
        // RCLCPP_INFO(this->get_logger(), "Size: %lu", lidarRanges.size());
        // RCLCPP_INFO(this->get_logger(), "opt Front: %f Left: %f Back: %f Right: %f", front, left, back, right);
        // RCLCPP_INFO(this->get_logger(), "raw Front: %f Left: %f Back: %f Right: %f", msg->ranges[0], msg->ranges[270], msg->ranges[540], msg->ranges[810]);
        // RCLCPP_INFO(this->get_logger(), "int Front: %f Left: %f Back: %f Right: %f", lidarRanges[0], lidarRanges[270], lidarRanges[540], lidarRanges[810]);
    }

// choose number of points for calculation or angle to measure - angle has higher priority
    double IoNode::getLidarRange(const double midAngle, int numOfPoints = 1, const int angle = 0) {
        double distance = 0.0;
        if (angle > 0 && angle < 360)
            numOfPoints = angle * 3; // 1080/360
        double ranges[numOfPoints];
        for (int i = 0; i < numOfPoints; i++) {
            int index = static_cast<int>(round(midAngle * 3.0)-floor(numOfPoints/2.0))+i;
            if (index < 0)
                index = lidarMaxIndex+index;
            else if (index > lidarMaxIndex)
                index = index-lidarMaxIndex;
            ranges[i] = lidarRanges[index];
        }
        std::sort(ranges, ranges + numOfPoints);
        if (numOfPoints%2 != 0) // liche cislo
            distance = ranges[(numOfPoints+1)/2];
        else // sude
            distance = (ranges[numOfPoints/2]+ranges[numOfPoints/2+1])/2.0;

        // for (int i = 0; i < numOfPoints; i++) {
            // printf("%f ", ranges[i]);
        // }
        // printf("\n");

        if (distance > maxWallDistance)
            distance = -1;
        else if (distance < minLidarDistance)
            distance = 0;

        return distance;
    }

    double IoNode::calculateRotation() {
        double beta = 8;
        double betaRad = beta/180*M_PI;
        int points = 5;

        double angle_left = 0;
        double angle_right = 0;

        bool valid_r = true;
        bool valid_l = true;

        double left_b = getLidarRange(90-beta, points, 0);
        double left_m = getLidarRange(90, points, 0);
        double left_f = getLidarRange(90+beta, points, 0);

        double right_b = getLidarRange(270+beta, points, 0);
        double right_m = getLidarRange(270, points, 0);
        double right_f = getLidarRange(270-beta, points, 0);

        if (right_b < 0 or right_m < 0 or right_f < 0)
            valid_r = false;

        if (left_b < 0 or left_m < 0 or left_f < 0)
            valid_l = false;

        if (valid_l) {
            double y_fml = sqrt(left_m*left_m + left_f*left_f - 2*left_f*left_m*cos(betaRad));
            double y_bml = sqrt(left_m*left_m + left_b*left_b - 2*left_b*left_m*cos(betaRad));

            double angle_lf = asin(left_f/y_fml*sin(betaRad))*180/M_PI-90;
            double angle_lb = 90-asin(left_b/y_bml*sin(betaRad))*180/M_PI;

            // RCLCPP_INFO(this->get_logger(), "front %f,  middle %f, back %f, front_dist %f, back_dist %f", left_f, left_m, left_b, y_fm, y_bm);
            // RCLCPP_INFO(this->get_logger(), "front %f,  back %f", angle_lf, angle_lb);

            if (angle_lb+angle_lf > 3)
                valid_l = false;

            angle_left = (angle_lb-angle_lf)/2;
        }

        if (valid_r) {
            double y_fmr = sqrt(right_m*right_m + right_f*right_f - 2*right_f*right_m*cos(betaRad));
            double y_bmr = sqrt(right_m*right_m + right_b*right_b - 2*right_b*right_m*cos(betaRad));

            double angle_rf = asin(right_f/y_fmr*sin(betaRad))*180/M_PI-90;
            double angle_rb = 90-asin(right_b/y_bmr*sin(betaRad))*180/M_PI;

            // RCLCPP_INFO(this->get_logger(), "front %f,  middle %f, back %f, front_dist %f, back_dist %f", right_f, right_m, right_b, y_fm, y_bm);
            // RCLCPP_INFO(this->get_logger(), "front %f,  back %f", angle_lf, angle_lb);

            if (angle_rb+angle_rf > 3)
                valid_r = false;

            angle_right = (angle_rb-angle_rf)/2;
        }
        if (valid_l and valid_r)
            return (angle_left + angle_right)/2;

        if (valid_l)
            return angle_left;

        if (valid_r)
            return angle_right;

        return 0;
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }


    // levy je 0, pravy je 1
    void IoNode::on_lineSensor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received LINE DATA \t Leva: %4d \t Prava: %4d", msg->data[0], msg->data[1]);

        lineLeft = msg->data[0];
        lineRight = msg->data[1];

        /*(msg->data[0] < 500 && msg->data[1] < 500)*/
    }

    double wheelDistance = 13.0;
    double wheelDiameter = 7.0;
    double wheelRadius = wheelDiameter / 2;
    double omega_r = 0.0;
    double omega_l = 0.0;
    double phi = 0.0;
    double robotPosition[2] = {0.0, 0.0};
    double robotRotation = 0.0;
    uint16_t pulsesPerRotation = 576;// 48*3*2*2
    double deltaEncoder[2] = {0.0, 0.0};
    int pValue[2] = {0, 0};

    RobotPose IoNode::calculatePositionAndRotation(const int encoderValues[2]) {
        if (pValue[0] == 0) {
            pValue[0] = encoderValues[0];
            pValue[1] = encoderValues[1];
        }
        deltaEncoder[0] = encoderValues[0] - pValue[0];
        deltaEncoder[1] = -(encoderValues[1] - pValue[1]);

        // RCLCPP_INFO(this->get_logger(), "dataLeft %f, dataRight %f", deltaEncoder[0], deltaEncoder[1]);

        omega_l =  deltaEncoder[0] / pulsesPerRotation * 2*M_PI;
        omega_r = deltaEncoder[1] / pulsesPerRotation * 2*M_PI;

        // RCLCPP_INFO(this->get_logger(), "omegaL %f, omegaR %f", omega_l, omega_r);

        double delta_x = wheelRadius * (omega_r + omega_l) * cos(robotRotation);
        double delta_y = wheelRadius * (omega_r + omega_l) * sin(robotRotation);
        double delta_phi = wheelRadius/wheelDistance * (omega_r - omega_l);

        // RCLCPP_INFO(this->get_logger(), "delta_x %f, delta_y %f, delta_phi %f", delta_x, delta_y, delta_phi);

        robotPosition[0] = robotPosition[0] + delta_x;
        robotPosition[1] = robotPosition[1] + delta_y;
        robotRotation = robotRotation + delta_phi;

        pValue[0] = encoderValues[0];
        pValue[1] = encoderValues[1];

        RobotPose pose;
        pose.pos_x = robotPosition[0];
        pose.pos_y = robotPosition[1];
        pose.angle = robotRotation*180/M_PI;

        return pose;
    }


}



    // double dt;
    // double max;
    // double min;
    // double Kp;
    // double Kd;
    // double Ki;
    // bool activeP = false;
    // bool activeI = false;
    // bool activeD = false;
    // double pre_error = 0;
    // double integral = 0;
    // double setpoint = 0;
    // double previousValue = 0;
    // double IoNode::calculatePID(int sensorValue, double* robotTransform) {
    //     // Calculate error
    //     double error = setpoint - previousValue;
    //
    //     // Proportional term
    //     double Pout = Kp * error;
    //
    //     // Integral term
    //     integral += error * dt;
    //     double Iout = Ki * integral;
    //
    //     // Derivative term
    //     double derivative = (error - pre_error) / dt;
    //     double Dout = Kd * derivative;
    //
    //     // Calculate total output
    //     double output = Pout*activeP + Iout*activeI + Dout*activeD;
    //
    //     // Restrict to max/min
    //     if( output > max )
    //         output = max;
    //     else if( output < min )
    //         output = min;
    //
    //     // Save error to previous error
    //     pre_error = error;
    //
    //     return output;


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

/*if (lineLeft < lineDetectionOffset && lineRight > lineDetectionOffset) {
    RCLCPP_INFO(this->get_logger(), "Going RIGHT");
    speedRight = 127;
} else if (lineLeft > lineDetectionOffset && lineRight < lineDetectionOffset) {
    RCLCPP_INFO(this->get_logger(), "Going LEFT");
    speedLeft = 127;
} else {
    RCLCPP_INFO(this->get_logger(), "Going STRAIGHT");
    speedLeft = maxSpeed;
    speedRight = maxSpeed;
}*/