#pragma once

#include <array>

#include <Eigen/Dense>

#include "joint_angles.hpp"
#include "quad_common.hpp"


// Take input turns and convert to scaled output radians
static constexpr double input_to_output_rad(double turns) {
    return turns * (2.0 * std::numbers::pi / quad::common::INPUT_TO_OUTPUT_SCALE);
}

// NOTE: All values follow SI units
//  - distance/length -> meters
//  - time -> seconds

namespace quad::config {
    // Swing proportional gains
    inline constexpr double alpha           = 0.5; // Positional
    inline constexpr double beta            = 0.5; // Rotational

    // Gait timings
    inline constexpr double overlap_time    = 0.10; // Duration where all feet on ground
    inline constexpr double swing_time      = 0.15; // Duration when only two diagonal feet on ground
    inline constexpr int overlap_ticks      = overlap_time / quad::common::DT;
    inline constexpr int swing_ticks        = swing_time / quad::common::DT;
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
    inline constexpr double ABDUCTION_OFFSET = 0.10300;
    inline constexpr double L1               = 0.19625;
    inline constexpr double L2               = 0.20356;
    inline constexpr double LEG_FB           = 0.30625;     // Front-back distance from center of body to hip joint axis of rotation
    inline constexpr double LEG_LR           = 0.11873; // Left-right distance from center of body to hip joint plane of rotation

    // Time step to clamp height correction speed
    inline constexpr double z_time_constant = 0.02;
    // Max clearance between body and foot in z direction
    inline constexpr double z_clearance_max = L1 + L2;
    // Min clearance between body and foot in z direction
    inline constexpr double z_clearance_min = L1;
    // z direction delta of swing height
    inline constexpr double z_delta_swing_height  = 0.07;


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
    inline const std::array<Eigen::Vector3d, quad::common::LEG_COUNT> DEFAULT_STANCE {
        Eigen::Vector3d{ LEG_FB,   ABDUCTION_OFFSET + LEG_LR , 0.0},
        Eigen::Vector3d{ LEG_FB, -(ABDUCTION_OFFSET + LEG_LR), 0.0},
        Eigen::Vector3d{-LEG_FB,   ABDUCTION_OFFSET + LEG_LR , 0.0},
        Eigen::Vector3d{-LEG_FB, -(ABDUCTION_OFFSET + LEG_LR), 0.0},
    };

    inline constexpr BodyJointAngles START_ANGLES = {{
    //                      hip_roll,                    hip_pitch,                  knee_pitch
        {input_to_output_rad(2.6225), input_to_output_rad( 2.3560), input_to_output_rad(-4.1744)}, // FL
        {input_to_output_rad(2.6104), input_to_output_rad(-2.1295), input_to_output_rad( 4.1919)}, // FR
        {input_to_output_rad(2.6510), input_to_output_rad(-2.3023), input_to_output_rad( 4.1599)}, // BL
        {input_to_output_rad(2.4992), input_to_output_rad( 2.3457), input_to_output_rad(-4.1717)}  // BR
    }};
};
