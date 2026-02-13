#pragma once

#include <iostream>

struct JointAngles {
    double hip_roll;
    double hip_pitch;
    double knee_pitch;
    static constexpr const char* IOX2_TYPE_NAME = "JointAngles";
};

inline auto operator<<(std::ostream& stream, const JointAngles& angle) -> std::ostream& {
    stream << "JointAngles { hip_roll: " << angle.hip_roll;
    stream << ", hip_pitch: "            << angle.hip_pitch;
    stream << ", knee_pitch: "           << angle.knee_pitch << " }";
    return stream;
}
