#pragma once

#include <array>

#include <Eigen/Dense>

#include "quad_common.hpp"

// NOTE: All values follow SI units
//  - distance/length -> meters
//  - time -> seconds

namespace quad::config {
    // Program Tick length
    inline constexpr double dt              = 0.01;
    inline constexpr int    dt_milli        = dt * 1E3;
    // Time step to clamp height correction speed
    inline constexpr double z_time_constant = 0.02;

    // Minimum clearance between body and foot in z direction
    inline constexpr double z_clearance     = 0.07;

    // Swing proportional gains
    inline constexpr double alpha           = 0.5; // Positional
    inline constexpr double beta            = 0.5; // Rotational

    // Gait timings
    inline constexpr double overlap_time    = 0.10; // Duration where all feet on ground
    inline constexpr double swing_time      = 0.15; // Duration when only two diagonal feet on ground
    inline constexpr int overlap_ticks      = overlap_time / dt;
    inline constexpr int swing_ticks        = swing_time / dt;
    inline constexpr int stance_ticks       = (2 * overlap_ticks) + swing_ticks;

    // Total tick duration of a full gait (swings and overlaps)
    inline constexpr int total_gait_ticks = (2 * overlap_ticks) + (2 * swing_ticks);

    // Contact modes
    inline constexpr int SWING  = 0;
    inline constexpr int STANCE = 1;

    inline constexpr int PHASE_COUNT = 4;
    // Phase order and amount of ticks in each phase
    inline constexpr std::array<int, PHASE_COUNT> phase_ticks = {
        overlap_ticks, swing_ticks, overlap_ticks, swing_ticks
    };

    // Foot contact mode (swing/stance) for each phase
    inline constexpr std::array<std::array<int, quad::common::LEG_COUNT>, PHASE_COUNT> contact_phases = {{
        {STANCE, STANCE,    // Overlap
         STANCE, STANCE},

        {STANCE, SWING ,    // Diagonal swing 1
         SWING , STANCE},

        {STANCE, STANCE,    // Overlap
         STANCE, STANCE},

        {SWING , STANCE,    // Diagonal swing 2
         STANCE, SWING }

        // Read as {FL, FR,
        //         {BL, BR}
    }};

    // Kinematic Lengths
    inline constexpr double ABDUCTION_OFFSET = 0.102;
    inline constexpr double L1               = 0.19625;
    inline constexpr double L2               = 0.140;
    inline constexpr double LEG_FB           = 0.35;     // Front-back distance from center of body to hip joint axis of rotation
    inline constexpr double LEG_LR           = 0.125725; // Left-right distance from center of body to hip joint plane of rotation

    // Abduction offsets per leg
    inline constexpr std::array<double, quad::common::LEG_COUNT> ABDUCTION_OFFSETS = {
         ABDUCTION_OFFSET,
        -ABDUCTION_OFFSET,
         ABDUCTION_OFFSET,
        -ABDUCTION_OFFSET,
    };

    // Origins of each legs hip relative to center of body
    inline const std::array<Eigen::Vector3d, quad::common::LEG_COUNT> LEG_HIP_ORIGINS {
        Eigen::Vector3d{ LEG_FB,  LEG_LR, 0},
        Eigen::Vector3d{ LEG_FB, -LEG_LR, 0},
        Eigen::Vector3d{-LEG_FB,  LEG_LR, 0},
        Eigen::Vector3d{-LEG_FB, -LEG_LR, 0},
    };

    // Default/Idle foot locations relative to center of body
    // TODO DR: Make the z location 0 for all
    inline constexpr double DEFAULT_Z = -(L1 + (L2 / 2));
    inline const std::array<Eigen::Vector3d, quad::common::LEG_COUNT> DEFAULT_STANCE {
        Eigen::Vector3d{ LEG_FB,   ABDUCTION_OFFSET + LEG_LR , DEFAULT_Z},
        Eigen::Vector3d{ LEG_FB, -(ABDUCTION_OFFSET + LEG_LR), DEFAULT_Z},
        Eigen::Vector3d{-LEG_FB,   ABDUCTION_OFFSET + LEG_LR , DEFAULT_Z},
        Eigen::Vector3d{-LEG_FB, -(ABDUCTION_OFFSET + LEG_LR), DEFAULT_Z},
    };
};
