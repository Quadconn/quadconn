#pragma once

#include "gamepad_data.hpp"

struct QuadCommand {
    static inline const double MAX_HORIZONTAL_VELOCITY_X = 0.3;
    static inline const double MAX_HORIZONTAL_VELOCITY_Y = 0.2;
    static inline const double MAX_YAW_RATE = 0.3;
    static inline const double MAX_HEIGHT_RATE = 0.1;

    void update(const GamepadData& gamepad) {
        horizontal_velocity_x = gamepad.ly * MAX_HORIZONTAL_VELOCITY_X;
        horizontal_velocity_y = gamepad.lx * MAX_HORIZONTAL_VELOCITY_Y;
        yaw_rate = gamepad.rx * MAX_YAW_RATE;

        if (gamepad.RB) {
            height_rate = (MAX_HEIGHT_RATE * GAMEPAD_DT);
        } else if (gamepad.LB) {
            height_rate = -(MAX_HEIGHT_RATE * GAMEPAD_DT);
        } else {
            height_rate = 0.0;
        }

        if (gamepad.Start && !prev_start_press) {
            is_trot_mode = !is_trot_mode;
        }
        prev_start_press = gamepad.Start;
    }

    double horizontal_velocity_x = 0.0;
    double horizontal_velocity_y = 0.0;
    double yaw_rate = 0.0;
    double height_rate = 0.0;
    bool   is_trot_mode = false;
    int    prev_start_press = 0;
};
