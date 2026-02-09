#pragma once

#include <iostream>

struct JointAngles {
    double hip_roll;
    double hip_pitch;
    double knee_pitch;
    static constexpr const char* IOX2_TYPE_NAME = "JointAngles";
};

struct Point {
    double x;
    double y;
    double z;
};


inline double sq(double a) {
    return a*a;
}

inline auto operator<<(std::ostream& stream, const JointAngles& value) -> std::ostream& {
    stream << "JointAngles { hip_roll: " << value.hip_roll;
    stream << ", hip_pitch: " << value.hip_pitch;
    stream << ", knee_pitch: " << value.knee_pitch << " }";
    return stream;
}

bool leg_ik(JointAngles& out, double x, double y, double z);

void leg_fk(Point& out, double hip_roll, double hip_pitch, double knee_pitch);
