#pragma once

#include <array>

#include <Eigen/Dense>

struct QuadConfig {
    static constexpr double dt              = 0.0;
    static constexpr double z_time_constant = 0.0;
    static constexpr double z_clearance     = 0.0;
    static constexpr double alpha           = 0.0;
    static constexpr double beta            = 0.0;
    static constexpr int stance_ticks       = 1;
    static constexpr int swing_ticks        = 1;
    static constexpr int overlap_ticks      = 1;
    static constexpr int num_legs           = 4;
    static constexpr int num_phases         = 4;
    static constexpr std::array<int, num_phases> phase_ticks = {
        overlap_ticks, swing_ticks, overlap_ticks, swing_ticks
    };
    static constexpr int phase_length = 2 * overlap_ticks + 2 * swing_ticks;

    static constexpr int FL = 0;
    static constexpr int FR = 1;
    static constexpr int BL = 2;
    static constexpr int BR = 3;

    static constexpr int STANCE = 1;
    static constexpr int SWING  = 0;

    static constexpr std::array<std::array<int, num_legs>, num_phases> contact_phases = {{
        {STANCE, STANCE, 
         STANCE, STANCE},

        {STANCE, SWING , 
         SWING , STANCE},

        {STANCE, STANCE, 
         STANCE, STANCE},

        {SWING , STANCE, 
         STANCE, SWING }
    }};

    // Kinematics
    static constexpr double ABDUCTION_OFFSET = 0.04241 + 0.069;
    static constexpr double L1  = 0.19425;
    static constexpr double L2  = 0.140;

    inline static const Eigen::Vector3d default_front_left_foot_location{0, 0, 0};
};
