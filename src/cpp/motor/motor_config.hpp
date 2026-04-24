#pragma once

#include "moteus.h"
#include "motor_diagnostics.hpp"
#include "quad_common.hpp"
#include "joint_angles.hpp"
#include <filesystem>
#include <array>
#include <memory>
#include <iostream>
#include <string>

#define MOTOR_NUM            12
#define KNEE_IDX             0
#define HIP_PITCH_IDX        1
#define HIP_ROLL_IDX         2

struct MotorDef {
    int can_id;
    std::shared_ptr<mjbots::moteus::Fdcanusb> bus;
    int leg;
    int joint_idx;
};

struct BusGroup {
    std::shared_ptr<mjbots::moteus::Fdcanusb> bus;
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    std::vector<int> legs;                                                
    std::vector<int> joint_types;                                         
    std::vector<mjbots::moteus::CanFdFrame> frames;                       
    std::vector<mjbots::moteus::CanFdFrame> replies;                      
};

// Make init_bus inline so it doesn't violate the One Definition Rule across files
inline std::shared_ptr<mjbots::moteus::Fdcanusb> init_bus(const std::string& port) {
    if (!std::filesystem::exists(port)) {
        std::cout << "[WARN] Hardware missing at " << port << ". Program will not run.\n";
        return nullptr;
    }
    
    try {
        return std::make_shared<mjbots::moteus::Fdcanusb>(port);
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Failed to init " << port << ": " << e.what() << "\n";
        return nullptr;
    }
}


// This returns a constant reference to the configuration. 
inline const std::array<MotorDef, MOTOR_NUM>& get_robot_config() {
    // Because these are static, they are initialized exactly ONCE, 
    // the first time this function is called.
    static const auto bus_red =    init_bus("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00"); 
    static const auto bus_yellow = init_bus("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00"); 
    static const auto bus_green =  init_bus("/dev/serial/by-id/usb-mjbots_fdcanusb_6A9ABCBD-if00"); 

    static const std::array<MotorDef, MOTOR_NUM> config = {{
    //can-id  wire        leg             joint
        {4 , bus_yellow, quad::common::FL, KNEE_IDX}, {3 , bus_yellow, quad::common::FL, HIP_PITCH_IDX}, {8, bus_green, quad::common::FL, HIP_ROLL_IDX},
        {2 , bus_yellow, quad::common::FR, KNEE_IDX}, {12, bus_yellow, quad::common::FR, HIP_PITCH_IDX}, {5, bus_green, quad::common::FR, HIP_ROLL_IDX},
        {11, bus_red   , quad::common::BL, KNEE_IDX}, {1 , bus_red   , quad::common::BL, HIP_PITCH_IDX}, {6, bus_green, quad::common::BL, HIP_ROLL_IDX},
        {10, bus_red   , quad::common::BR, KNEE_IDX}, {9 , bus_red   , quad::common::BR, HIP_PITCH_IDX}, {7, bus_green, quad::common::BR, HIP_ROLL_IDX }
    }};

    return config;
}


inline double parse_angle(int leg, int joint_idx, const BodyJointAngles& target) {
    // retrieve index of leg joint, as well as specific leg
    const auto& leg_data = target.body_joint_angles[leg];
    switch (joint_idx) {
        case(KNEE_IDX): 
            return leg_data.knee_pitch;
        case(HIP_PITCH_IDX):
            return leg_data.hip_pitch;
        case(HIP_ROLL_IDX):
            return leg_data.hip_roll;
        default:
            return 0.0;
    }
}
