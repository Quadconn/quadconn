#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>
#include <filesystem>
#include <stdexcept>

#include "moteus.h"
#include "joint_angles.hpp"
#include "quad_common.hpp"
#include "motor_diagnostics.hpp"
#include "quad_ipc.hpp"

#include "system_logic.hpp"
#include "iox2/iceoryx2.hpp"

#define UPDATE_RATE_MS   5
#define MOTOR_NUM        12

#define BUS_CTRL_NUM     4

std::shared_ptr<mjbots::moteus::Fdcanusb> init_bus(const std::string& port);

// used to define mapping
struct MotorDef {
    int can_id;
    std::shared_ptr<mjbots::moteus::Fdcanusb> bus;
    int leg;
    int joint_idx;
};

// used to construct sending/receiving messages
// each bus will contain a vector of its own controllers, legs, joint types, and frames
struct BusGroup {
    std::shared_ptr<mjbots::moteus::Fdcanusb> bus;
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    std::vector<int> legs;          
    std::vector<int> joint_types;   
    std::vector<mjbots::moteus::CanFdFrame> frames;
};

inline double parse_angle(int leg, int joint_idx, const BodyJointAngles& target) {
    // retrieve index of leg joint, as well as specific leg
    const auto& leg_data = target.body_joint_angles[leg];
    switch (joint_idx) {
        case(0): 
            return leg_data.hip_roll;
        case(1):
            return leg_data.hip_pitch;
        case(2):
            return leg_data.knee_pitch;
        default:
            return 0.0;
    }
}




int main(int argc, char** argv) {
    using namespace mjbots;
    using namespace iox2;

    
    // change usb-id depending on motor id used, map motor controller node ids to bus
    try {
        
    } catch(std::runtime_error) {
        std::cout << "bus_a is not connected\n";
    }
    const auto bus_a = init_bus("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00");    
    const auto bus_b = init_bus("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00");
    const auto bus_c = init_bus("/dev/serial/by-id/usb-mjbots_fdcanusb_[INSERTCODE]-if00");

    
    std::array<MotorDef, MOTOR_NUM> robot_config = {{
        // implicitly tie joint_idx = 2 -> knee, joint_idx = 1 -> hip_pitch, joint_idx = 0 -> hip_roll
        {1,  bus_b, quad::common::FR, 0}, {2,  bus_b, quad::common::FR, 1}, {3,  bus_a, quad::common::FR, 2},
        {4,  bus_b, quad::common::FL, 0}, {5,  bus_b, quad::common::FL, 1}, {6,  bus_a, quad::common::FL, 2},
        {7,  bus_c, quad::common::BL, 0}, {8,  bus_c, quad::common::BL, 1}, {9,  bus_a, quad::common::BL, 2},
        {10, bus_c, quad::common::BR, 0}, {11, bus_c, quad::common::BR, 1}, {12, bus_a, quad::common::BR, 2}
    }};

    /* START: BRACKET GUARD -- Init Node */
    auto motor_node = make_node("motor_node");

    auto diagnostic_publisher = make_publisher<MotorDiagnosticsArray>
        (make_service<MotorDiagnosticsArray>("MotorDiagnosticsArray", motor_node));
    auto angle_subscriber = make_subscriber<BodyJointAngles>
        (make_service<BodyJointAngles>("BodyJointAngles", motor_node));
    auto system_listener = make_listener
        (make_event("SystemLogic", motor_node));
    const bb::Duration node_duration = bb::Duration::from_millis(UPDATE_RATE_MS);
    /* END: BRACKET GUARD -- Init Node */


    std::map<std::shared_ptr<moteus::Fdcanusb>, BusGroup> bus_groups_map;

    for (const auto& def : robot_config) {
        moteus::Controller::Options opts{};
        opts.id = def.can_id;
        opts.transport = def.bus;

        // populate BusGroup directly using key-value pairs
        // if already exists, then retrieve reference. Otherwise, create
        // another key-value pair
        auto& group = bus_groups_map[def.bus];

        group.bus = def.bus;
        // populate vectors once
        group.controllers.push_back(std::make_shared<moteus::Controller>(opts));
        group.legs.push_back(def.leg);
        group.joint_types.push_back(def.joint_idx);
        
        // Pre-allocate empty frame to avoid resizing in the hot loop
        group.frames.push_back(moteus::CanFdFrame()); 
    }

    // index bus_groups to iterate through controllers/commands.
    // will now use bus_groups for all communication (no more map)
    std::vector<BusGroup> bus_groups;
    for (auto& [bus, group] : bus_groups_map) {
        bus_groups.push_back(std::move(group));
    }


    // Clear faults
    for(auto& group : bus_groups) {
        for(auto& c : group.controllers) c->SetStop();
    }

    // Prepare batch of commands
    BodyJointAngles target_val{};
    // handles moteus sending, query, and receiving data structures
    moteus::PositionMode::Command cmd;
    // TODO: change maps and vectors to static arrays 
    std::vector<moteus::CanFdFrame> replies;
    replies.reserve(MOTOR_NUM);

    // Execution loop
    std::cout << "starting motor loop\n";
    // attempt to receive value on loop duration
    while (loop_waitms(UPDATE_RATE_MS, motor_node)) { 
        
        auto event = system_listener.try_wait_one();
        // catch stop signal
        if (event.has_value()) {
            if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::KillMotors) {
                    std::cout << "stopping motor_controller process";
                    break;
            }
        }
        

        // receiving values from JointAngles struct
        target_val = ipc_receive(angle_subscriber).value_or(target_val);

        replies.clear();

        //  build the CAN frames using the latest targets 
        //  grouped by buses
        for (auto& group : bus_groups) {
            
            // Build frames for just this specific bus
            for (size_t i = 0; i < group.controllers.size(); ++i) {
                double angle_cmd = parse_angle(group.legs[i], group.joint_types[i], target_val);
                cmd.position = rad2turns(angle_cmd);
                cmd.velocity = std::numeric_limits<double>::quiet_NaN(); 
                group.frames[i] = group.controllers[i]->MakePosition(cmd);
            }
        }


        // Loan default-constructed memory directly from the shared pool
        auto sample_s_opt = diagnostic_publisher.loan();
        if (sample_s_opt.has_value()) {
            auto sample_s = std::move(sample_s_opt.value());
            
            for (const auto& frame : replies) {
                auto result = moteus::Query::Parse(frame.data, frame.size);
                int target_idx = frame.source - 1; 
                
                if (target_idx >= 0 && target_idx < MOTOR_NUM) {
                    sample_s.payload_mut().motor_instance[target_idx] = motor_info::make_diag(result);
                }
            }
            // Send the sample off to any subscribers
            iox2::send(std::move(sample_s)).value();
        }
    }
    // End safely,
    // Clear faults
    for (auto& group : bus_groups) {
        for (auto& c : group.controllers) { c->SetStop(); }
    }
    return 0;
}



// Attempts to open the CAN bus. Returns nullptr if it fails or is missing.
std::shared_ptr<mjbots::moteus::Fdcanusb> init_bus(const std::string& port) {
    if (!std::filesystem::exists(port)) {
        std::cout << "[WARN] Hardware missing at " << port << ". Defaulting to Simulation Mode.\n";
        return nullptr;
    }
    
    try {
        return std::make_shared<mjbots::moteus::Fdcanusb>(port);
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Failed to init " << port << ": " << e.what() << "\n";
        return nullptr; // Fallback to simulation
    }
}