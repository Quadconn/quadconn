#pragma once

#include "gamepad_data.hpp"
#include "quad_config.hpp"

struct QuadCommand {

    void update(const GamepadData& gamepad) {
        horizontal_velocity_x = gamepad.ly * quad::config::MAX_HORIZONTAL_VELOCITY_X;
        horizontal_velocity_y = gamepad.lx * quad::config::MAX_HORIZONTAL_VELOCITY_Y;
        yaw_rate = gamepad.rx * quad::config::MAX_YAW_RATE;

        if (gamepad.RB) {
            height_rate = (quad::config::MAX_HEIGHT_RATE * GAMEPAD_DT);
        } else if (gamepad.LB) {
            height_rate = -(quad::config::MAX_HEIGHT_RATE * GAMEPAD_DT);
        } else {
            height_rate = 0.0;
        }

        is_toggle_mode = (gamepad.A && !prev_a_press)? true : false;
        prev_a_press = gamepad.A;

        is_toggle_shutdown = (gamepad.Start && !prev_start_press)? true : false;
        prev_start_press = gamepad.Start;
    }

    double horizontal_velocity_x = 0.0;
    double horizontal_velocity_y = 0.0;
    double yaw_rate = 0.0;
    double height_rate = 0.0;
    bool   is_toggle_mode = false;
    int    prev_a_press = 0;
    bool   is_toggle_shutdown = false;
    int    prev_start_press = 0;
};
