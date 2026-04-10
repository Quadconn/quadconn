#pragma once

#include <iostream>
#include <limits>
#include "quad_common.hpp"

struct LegJointAngles {
    double hip_roll = std::numeric_limits<double>::quiet_NaN();
    double hip_pitch = std::numeric_limits<double>::quiet_NaN();
    double knee_pitch = std::numeric_limits<double>::quiet_NaN();
    static constexpr const char* IOX2_TYPE_NAME = "LegJointAngles";
};

inline auto operator<<(std::ostream& stream, const LegJointAngles& angle) -> std::ostream& {
    stream << "LegJointAngles { hip_roll: " << angle.hip_roll;
    stream << ", hip_pitch: "               << angle.hip_pitch;
    stream << ", knee_pitch: "              << angle.knee_pitch << " }";
    return stream;
}

struct BodyJointAngles {
    LegJointAngles body_joint_angles[quad::common::LEG_COUNT];
    static constexpr const char* IOX2_TYPE_NAME = "BodyJointAngles";
};

inline auto operator<<(std::ostream& stream, const BodyJointAngles& BodyJoint) -> std::ostream& {
    
    for (int i = 0; i < quad::common::LEG_COUNT; i++) {
            stream << "BodyJointAngles {Leg " <<  i << "--"
            << " hip_roll: " << BodyJoint.body_joint_angles[i].hip_roll
            << " hip_pitch: " << BodyJoint.body_joint_angles[i].hip_pitch 
            << "knee_pitch: " << BodyJoint.body_joint_angles[i].knee_pitch << " }\n";
        }
    return stream;
}