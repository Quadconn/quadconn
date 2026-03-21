#pragma once

#include "gamepad_data.hpp"

struct QuadCommand {
    static inline const double MAX_HORIZONTAL_VELOCITY_X = 0.4;
    static inline const double MAX_HORIZONTAL_VELOCITY_Y = 0.3;
    static inline const double MAX_YAW_RATE = 0.3;

    void update(const GamepadData& gamepad) {
        horizontal_velocity_x = gamepad.ly * MAX_HORIZONTAL_VELOCITY_X;
        horizontal_velocity_y = gamepad.lx * MAX_HORIZONTAL_VELOCITY_Y;
        yaw_rate = gamepad.rx * MAX_YAW_RATE;
    }

    double horizontal_velocity_x = 0.0;
    double horizontal_velocity_y = 0.0;
    double yaw_rate = 0.0;
    double height = 0.0;
};
