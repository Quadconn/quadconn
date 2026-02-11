#pragma once

#include <iostream>

struct Command {
    double horizontal_velocity_x;
    double horizontal_velocity_y;
    double yaw_rate;
    double height;
    static constexpr const char* IOX2_TYPE_NAME = "Command";
};

// TODO DR: Finish definition of this.
inline auto operator<<(std::ostream& stream, const Command& value) -> std::ostream& {
    return stream;
}
