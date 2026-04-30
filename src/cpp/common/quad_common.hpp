#pragma once

namespace quad::common {
    // Timing
    inline constexpr double DT        = 0.01;
    inline constexpr int    DT_MILLI  = DT * 1E3;

    // Array sizes
    inline constexpr int LEG_COUNT = 4;

    // Array Indexing per Leg
    inline constexpr int FL = 0; // Front left
    inline constexpr int FR = 1; // Front right
    inline constexpr int BL = 2; // Back left
    inline constexpr int BR = 3; // Back right
    
    // Input to output scaling factor
    inline constexpr double INPUT_TO_OUTPUT_SCALE = 9.0;

    inline constexpr double FL_KNEE_0      = -4.1062; // CAN-ID 4
    inline constexpr double FL_HIP_PITCH_0 =  2.2899; // CAN-ID 9
    inline constexpr double FL_HIP_ROLL_0  = -2.6163; // CAN-ID 5
    inline constexpr double FR_KNEE_0      =  4.0685; // CAN-ID 3 
    inline constexpr double FR_HIP_PITCH_0 = -2.2288; // CAN-ID 2
    inline constexpr double FR_HIP_ROLL_0  = -2.6619; // CAN-ID 7
    inline constexpr double BL_KNEE_0      =  4.0120; // CAN-ID 1
    inline constexpr double BL_HIP_PITCH_0 = -2.3069; // CAN-ID 11
    inline constexpr double BL_HIP_ROLL_0  =  2.6256; // CAN-ID 8
    inline constexpr double BR_KNEE_0      = -4.0993; // CAN-ID 12
    inline constexpr double BR_HIP_PITCH_0 =  2.2881; // CAN-ID 10
    inline constexpr double BR_HIP_ROLL_0  = -2.6750; // CAN-ID 6
};

