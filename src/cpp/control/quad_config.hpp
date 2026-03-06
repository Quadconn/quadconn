#pragma once

#include <array>

#include <Eigen/Dense>

// NOTE: All values follow SI units
//  - distance/length -> meters
//  - time -> seconds

struct QuadConfig {
    // Array sizes
    static constexpr int LEG_COUNT   = 4;
    static constexpr int PHASE_COUNT = 4;

    // Program Tick length
    static constexpr double dt              = 0.01;
    static constexpr int    dt_milli        = dt * 1E3;
    // Time step to clamp height correction speed
    static constexpr double z_time_constant = 0.02;

    // Minimum clearance between body and foot in z direction
    static constexpr double z_clearance     = 0.07;

    // Swing proportional gains
    static constexpr double alpha           = 0.5; // Positional
    static constexpr double beta            = 0.5; // Rotational

    // Gait timings
    static constexpr double overlap_time    = 0.10; // Duration where all feet on ground
    static constexpr double swing_time      = 0.15; // Duration when only two diagonal feet on ground
    static constexpr int overlap_ticks      = overlap_time / dt;
    static constexpr int swing_ticks        = swing_time / dt;
    static constexpr int stance_ticks       = (2 * overlap_ticks) + swing_ticks;

    // Total tick duration of a full gait (swings and overlaps)
    static constexpr int total_gait_ticks = (2 * overlap_ticks) + (2 * swing_ticks);

    // Array Indexing
    static constexpr int FL = 0; // Front left
    static constexpr int FR = 1; // Front right
    static constexpr int BL = 2; // Back left
    static constexpr int BR = 3; // Back right

    // Contact modes
    static constexpr int SWING  = 0;
    static constexpr int STANCE = 1;

    // Phase order and amount of ticks in each phase
    static constexpr std::array<int, PHASE_COUNT> phase_ticks = {
        overlap_ticks, swing_ticks, overlap_ticks, swing_ticks
    };

    // Foot contact mode (swing/stance) for each phase
    static constexpr std::array<std::array<int, LEG_COUNT>, PHASE_COUNT> contact_phases = {{
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
    static constexpr double ABDUCTION_OFFSET = 0.102;
    static constexpr double L1  = 0.19625;
    static constexpr double L2  = 0.140;

    // Default/Idle foot location
    inline static const Eigen::Vector3d default_front_left_foot_location{0, ABDUCTION_OFFSET, -(L1 + (L2 / 2))};
};
