#pragma once

#include <iostream>

struct JointAngles {
    double hip_roll;
    double hip_pitch;
    double knee_pitch;
    static constexpr const char* IOX2_TYPE_NAME = "JointAngles";
};

inline auto operator<<(std::ostream& stream, const JointAngles& value) -> std::ostream& {
    stream << "JointAngles { hip_roll: " << value.hip_roll;
    stream << ", hip_pitch: "            << value.hip_pitch;
    stream << ", knee_pitch: "           << value.knee_pitch << " }";
    return stream;
}
