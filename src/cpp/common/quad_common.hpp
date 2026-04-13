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
};
