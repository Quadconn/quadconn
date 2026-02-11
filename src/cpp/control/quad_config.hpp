#pragma once

#include <Eigen/Dense>

struct QuadConfig {
    static constexpr double dt              = 0.0;
    static constexpr double z_time_constant = 0.0;
    static constexpr double z_clearance     = 0.0;
    static constexpr double alpha           = 0.0;
    static constexpr double beta            = 0.0;
    static constexpr int stance_ticks       = 0;
    static constexpr int swing_ticks        = 0;

    // Kinematics
    static constexpr double ABDUCTION_OFFSET = 0.04241 + 0.069;
    static constexpr double L1  = 0.19425;
    static constexpr double L2  = 0.140;

    inline static const Eigen::Vector3d default_front_left_foot_location{0, 0, 0};
};
