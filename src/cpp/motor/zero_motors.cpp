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


int main(int argc, char** argv) {
    using namespace mjbots;

    // Define initial positions, indexed as N+1 as nodes
    std::array<double, MOTOR_COUNT> zero_positions = 
    {rad2turns(-2.302),               rad2turns(4.192),      rad2turns(2.356),
     rad2turns(-4.174),               rad2turns(2.610),      rad2turns(2.651),
     rad2turns(2.500),                rad2turns(2.623),      rad2turns(2.346),
     rad2turns(-4.172),               rad2turns(4.160),      rad2turns(-2.129)};

    // Initialize Controllers
    std::cout << "initializing motors\n";
    std::map<std::shared_ptr<moteus::Fdcanusb>, BusGroup> bus_groups_map;

    for (const auto& def : get_robot_config()) {
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
    
    std::vector<BusGroup> bus_groups;
    for (auto& [bus, group] : bus_groups_map) {
        if (bus == nullptr) {
            std::cout << "bus in warning not connnected, program exiting early\n";
            return 1;
        }
        bus_groups.push_back(std::move(group));
    }

    std::cout << "clearing faults\n";
    // Stop everything to clear faults first
    for (auto& group : bus_groups) {
        for(size_t i = 0; i < group.controllers.size(); ++i) {
            group.frames[i] = group.controllers[i]->MakeStop();
        }
        group.replies.clear();
        group.bus->BlockingCycle(group.frames.data(), group.frames.size(), &group.replies);
    }


    // Execution loop
    std::cout << "starting calibration sequence\n";
    
    for (auto& group : bus_groups) {
        // Build frames for just this specific bus
        for (size_t i = 0; i < group.controllers.size(); ++i) {
                                            // legs and joint_types are indexable by integer
            int id = group.controllers[i]->options().id;
            int idx = id - 1;
            std::string exact_cmd = "d exact " + std::to_string(zero_positions[idx]);
            // d cfg-set-output: updates the 'motor_position.sources.0.offset' config
            std::string sync_cmd = "d cfg-set-output " + std::to_string(zero_positions[idx]);
            try {
                std::cout << "Calibrating Motor " << id << " to " << zero_positions[idx]*(GEAR_RATIO/M_PI) 
                          << " radians...\n";
                
                // Execute the sequence
                group.controllers[i]->DiagnosticCommand(exact_cmd);    // Set runtime position
                group.controllers[i]->DiagnosticCommand(sync_cmd);     // Sync config offset
                group.controllers[i]->DiagnosticCommand("conf write"); // Commit to non-volatile flash
                
                std::cout << "Motor " << id << " saved successfully.\n";
            } catch (const std::exception& e) {
                std::cerr << "Failed to calibrate motor " << id << ": " << e.what() << "\n";
            }
        }
        // clear all data before writing replies
        group.replies.clear();
        group.bus->BlockingCycle(group.frames.data(), group.frames.size(), &group.replies);
    }

            
    // End safely
    for (auto& group : bus_groups) {
        for(size_t i = 0; i < group.controllers.size(); ++i) {
            group.frames[i] = group.controllers[i]->MakeStop();
        }
        group.replies.clear();
        group.bus->BlockingCycle(group.frames.data(), group.frames.size(), &group.replies);
    }
    return 0;
}

