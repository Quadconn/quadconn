#pragma once

#include <iostream>

struct Command {
    double horizontal_velocity_x;
    double horizontal_velocity_y;
    double yaw_rate;
    double height;
    static constexpr const char* IOX2_TYPE_NAME = "Command";
};

inline auto operator<<(std::ostream& stream, const Command& cmd) -> std::ostream& {
    stream << "Command { horizontal_velocity x|y: " << cmd.horizontal_velocity_x
           << "|" << cmd.horizontal_velocity_y;
    stream << ", yaw_rate :" << cmd.yaw_rate;
    stream << ", height :" << cmd.height;
    return stream;
}
