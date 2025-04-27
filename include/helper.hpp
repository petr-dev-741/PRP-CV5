#pragma once

#include <iostream>

//static const int MAIN_LOOP_PERIOD_MS = 50;

namespace Topic {
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
    const std::string encoders = "/bpc_prp_robot/encoders";
    const std::string motor_speeds = "/bpc_prp_robot/set_motor_speeds";
    const std::string line_sensors = "/bpc_prp_robot/line_sensors";
    const std::string imu = "/bpc_prp_robot/imu";
    const std::string lidar = "/bpc_prp_robot/lidar";
    const std::string ultrasounds = "/bpc_prp_robot/ultrasounds";
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};
