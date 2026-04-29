#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>

#include "moteus.h"
#include "motor_diagnostics.hpp"
#include "motor_config.hpp"
#include "quad_common.hpp"

int main(int argc, char** argv) {

    // Define initial positions, indexed as N+1 as nodes
    std::array<double, MOTOR_COUNT> zero_positions = 
    {quad::common::BL_HIP_PITCH_0,  quad::common::FR_KNEE_0,     quad::common::FL_HIP_PITCH_0, 
     quad::common::FL_KNEE_0,       quad::common::FR_HIP_ROLL_0, quad::common::BL_HIP_ROLL_0,
     quad::common::BR_HIP_ROLL_0,   quad::common::FL_HIP_ROLL_0, quad::common::BR_HIP_PITCH_0,
     quad::common::BR_KNEE_0,       quad::common::BL_KNEE_0,     quad::common::FR_HIP_PITCH_0
    };

    // Initialize Controllers
    std::cout << "initializing motors\n";


    // Execution loop
    std::cout << "starting calibration sequence\n";
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Build frames for just this specific bus
        int id = i;
        int idx = id - 1;
        std::string exact_cmd = "d exact " + std::to_string(zero_positions[idx]);
        // d cfg-set-output: updates the 'motor_position.sources.0.offset' config
        std::string sync_cmd = "d cfg-set-output " + std::to_string(zero_positions[idx]);
        try {
            std::cout << "Calibrating Motor " << id << " to " << zero_positions[idx] << "turns\n";
        
            
            std::cout << "Motor " << id << " saved successfully.\n";
        } catch (const std::exception& e) {
            std::cerr << "Failed to calibrate motor " << id << ": " << e.what() << "\n";
        }
    }
    

            

    return 0;
}

