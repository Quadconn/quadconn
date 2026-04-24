#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>
#include <filesystem>
#include <stdexcept>

#include "joint_angles.hpp"
#include "motor_diagnostics.hpp"
#include "quad_ipc.hpp"
#include "motor_config.hpp"
#include "system_logic.hpp"
#include "iox2/iceoryx2.hpp"

#define ACCEL_LIMIT 2000
#define KP_SCALE    1.0
int main(int argc, char** argv) {
    using namespace mjbots;
    using namespace iox2;

    /* START: BRACKET GUARD -- Init Node */
    auto motor_node = make_node("motor_node");

    auto diagnostic_publisher = make_publisher<MotorDiagnosticsArray>
        (make_service<MotorDiagnosticsArray>("MotorDiagnosticsArray", motor_node));
    auto angle_subscriber = make_subscriber<BodyJointAngles>
        (make_service<BodyJointAngles>("BodyJointAngles", motor_node));
    auto system_listener = make_listener
        (make_event("SystemLogic", motor_node));

    /* END: BRACKET GUARD -- Init Node */


    std::map<std::shared_ptr<moteus::Fdcanusb>, BusGroup> bus_groups_map;

    for (const auto& def : get_robot_config()) {
        moteus::Controller::Options opts{};
        opts.id = def.can_id;
        opts.transport = def.bus;
        // disable the following replies
        opts.query_format.mode =        moteus::Resolution::kIgnore;
        opts.query_format.position =    moteus::Resolution::kIgnore;
        opts.query_format.velocity =    moteus::Resolution::kIgnore;
        // enable the following replies
        opts.query_format.voltage =     moteus::Resolution::kInt8;
        opts.query_format.torque =      moteus::Resolution::kFloat; 
        opts.query_format.power =       moteus::Resolution::kFloat;  
        opts.query_format.fault =       moteus::Resolution::kInt8;
        opts.query_format.temperature = moteus::Resolution::kInt8;

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
        if (bus == nullptr) {
            std::cout << "bus in warning not connnected, program exiting early\n";
            return 1;
        }
        bus_groups.push_back(std::move(group));
    }

    std::cout << "waiting for control code\n";
    while (true) {
        auto event = system_listener.blocking_wait_one();
        if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::StartMotors) {
            std::cout << "starting motor_controller process\n";
            break;
        }
    }

    // Clear faults
    for (auto& group : bus_groups) {
        for(size_t i = 0; i < group.controllers.size(); ++i) {
            group.frames[i] = group.controllers[i]->MakeStop();
        }
        group.replies.clear();
        group.bus->BlockingCycle(group.frames.data(), group.frames.size(), &group.replies);
    }

    // Prepare batch of commands
    BodyJointAngles target_val{};
    //     

    // handles moteus sending, query, and receiving data structures
    moteus::PositionMode::Command cmd;

    // Execution loop
    std::cout << "starting motor loop\n";
    // attempt to receive value, will wait until events occur
    while (true) { 
        
        auto event = system_listener.blocking_wait_one();
        // catch stop signal
        if (event.has_value() && event.value().has_value()) {
            if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::KillMotors) {
                std::cout << "stopping motor_controller process";
                break;
            } else
            if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::QuadControlDone) {
                        // receiving values from JointAngles struct
                target_val = ipc_receive(angle_subscriber).value_or(target_val);

                //  build the CAN frames using the latest targets 
                //  grouped by buses
                for (auto& group : bus_groups) {
                    // Build frames for just this specific bus
                    for (size_t i = 0; i < group.controllers.size(); ++i) {
                                                    // legs and joint_types are indexable by integer
                        double angle_cmd = parse_angle(group.legs[i], group.joint_types[i], target_val);
                        cmd.position = rad2turns(angle_cmd);
                        cmd.velocity = std::numeric_limits<double>::quiet_NaN();
                        cmd.accel_limit = ACCEL_LIMIT; 
                        cmd.kp_scale = KP_SCALE;
                        // index vector like array (avoid push_back resizing)
                        group.frames[i] = group.controllers[i]->MakePosition(cmd);
                    }
                    // clear all data before writing replies
                    group.replies.clear();
                    group.bus->BlockingCycle(group.frames.data(), group.frames.size(), &group.replies);
                }
                
                // Loan default-constructed memory directly from the shared pool
                auto sample_s_opt = diagnostic_publisher.loan();
                if (sample_s_opt.has_value()) {
                    auto sample_s = std::move(sample_s_opt.value());

                    for (const auto& group : bus_groups) {
                        for (const auto& frame: group.replies) {
                            auto result = moteus::Query::Parse(frame.data, frame.size);
                            int target_idx = frame.source - 1; 
                            
                            if (target_idx >= 0 && target_idx < MOTOR_NUM) {
                                motor_info::populate_diag(
                                    result, 
                                    sample_s.payload_mut().motor_instance[target_idx]);
                            }
                        }
                    }
                    // Send the sample off to any subscribers
                    iox2::send(std::move(sample_s)).value();
                    
                }
            } else {
                for (auto& group : bus_groups) {
                    for (size_t i = 0; i < group.controllers.size(); ++i) {
                        cmd.position = std::numeric_limits<double>::quiet_NaN();
                        cmd.velocity = std::numeric_limits<double>::quiet_NaN(); 
                        group.frames[i] = group.controllers[i]->MakePosition(cmd);
                    }
                    // clear all data before writing replies
                    group.replies.clear();
                    group.bus->BlockingCycle(group.frames.data(), group.frames.size(), &group.replies);
                }
            }
                
        }
    }

    // End safely,
    // Clear faults
    for (auto& group : bus_groups) {
        for(size_t i = 0; i < group.controllers.size(); ++i) {
            group.frames[i] = group.controllers[i]->MakeStop();
        }
        group.replies.clear();
        group.bus->BlockingCycle(group.frames.data(), group.frames.size(), &group.replies);
    }
    return 0;
}
