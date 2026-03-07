#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>

#include "moteus.h"
#include "joint_angles.hpp"
#include "quad_common.hpp"
#include "motor_diagnostics.hpp"
#include "quad_ipc.hpp"


#include "iox2/iceoryx2.hpp"

#define UPDATE_RATE_MS   5
#define MOTOR_NUM        6

inline double parse_angle(int index, const BodyJointAngles& target_val) {
    switch (index) {
        case(0):
            return target_val.body_joint_angles[quad::common::FR].hip_roll;
        case(1):
            return target_val.body_joint_angles[quad::common::FR].hip_pitch;
        case(2):
            return target_val.body_joint_angles[quad::common::FR].knee_pitch;
        case(3):
            return target_val.body_joint_angles[quad::common::FL].hip_roll;
        case(4):
            return target_val.body_joint_angles[quad::common::FL].hip_pitch;
        case(5):
            return target_val.body_joint_angles[quad::common::FL].knee_pitch;
        default:
            return 0.0;
    }

}

// struct id_fdcan {
//     int id;
//     std::shared_ptr<mjbots::moteus::Fdcanusb> bus;
// };

// std::array<id_fdcan, MOTOR_NUM> id_to_bus {{
//     {1, bus_a},
//     {2, bus_a},
//     {3, bus_a},
//     {4, bus_b},
//     {5, bus_b},
//     {6, bus_b}
// }};

int main(int argc, char** argv) {
    using namespace mjbots;
    using namespace iox2;

    
    
    // change usb-id depending on motor id used, map motor controller node ids to bus
    const auto bus_a = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00");
    const auto bus_b = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00");
    // const auto bus_c = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_[INSERTCODE]-if00");


    std::map<int, std::shared_ptr<moteus::Fdcanusb>> id_to_bus = {
        {1, bus_b}, // hip_roll   FR - 1
        {2, bus_b}, // hip_pitch  FR - 1
        {3, bus_b}, //  hip_knee  FR - 1
        {4, bus_a}, // hip_roll   FL - 0
        {5, bus_a}, // hip_pitch  FL - 0
        {6, bus_a}  // hip_knee   FL - 0

        // {7, bus_a},
        // {8, bus_b},
        // {9, bus_c},
        // {10,bus_c},
        // {11,bus_c},
    };

    /* START: BRACKET GUARD -- Init Node */
    auto motor_node = make_node("motor_node");

    auto diagnostic_publisher = make_publisher<MotorDiagnosticsArray>
        (make_service<MotorDiagnosticsArray>("MotorDiagnosticsArray", motor_node));
    auto angle_subscriber = make_subscriber<BodyJointAngles>
        (make_service<BodyJointAngles>("BodyJointAngles", motor_node));

    const bb::Duration node_duration = bb::Duration::from_millis(UPDATE_RATE_MS);
    /* END: BRACKET GUARD -- Init Node */

    // Initialize Controllers
    std::array<std::shared_ptr<moteus::Controller>, MOTOR_NUM> controllers;
    {
        int i = 0; 
        for (auto const& [id, bus] : id_to_bus) {
            moteus::Controller::Options options{};
            options.id = id;
            options.transport = bus; 
            controllers[i] = std::make_shared<moteus::Controller>(options);
            i++;
        }
    }
    // Stop everything to clear faults first
    for (auto& c : controllers) { c->SetStop(); }


    // Prepare batch of commands
    BodyJointAngles target_val{};

    // handles moteus sending, query, and receiving data structures
    moteus::PositionMode::Command cmd;
    // TODO: change maps and vectors to static arrays 
    std::vector<moteus::CanFdFrame> replies;
    std::map<int, moteus::Query::Result> servo_data;
    // moteus transport information
    std::map<std::shared_ptr<moteus::Fdcanusb>, std::vector<moteus::CanFdFrame>> bus_cmd;

    // Execution loop
    std::cout << "starting motor loop\n";
    
    // attempt to receive value on loop duration
    while (loop_waitms(UPDATE_RATE_MS, motor_node)) { 
        
        // clear on new loop
        servo_data.clear();
        bus_cmd.clear();
        replies.clear();

        // receiving values from JointAngles struct
        target_val = ipc_receive(angle_subscriber).value_or(target_val);

        std::cout << target_val;
        //  build the CAN frames using the latest targets
        for (size_t i = 0; i < controllers.size(); ++i) {
            double angle_cmd = parse_angle(i, target_val);
            cmd.position = rad2turns(angle_cmd);
            cmd.velocity = std::numeric_limits<double>::quiet_NaN(); 
            auto frame = controllers[i]->MakePosition(cmd);
            // for commmands, map CAN-ID with frame
            bus_cmd[id_to_bus[controllers[i]->options().id]].push_back(frame);
        }

        
        //  Execute a cycle for EACH transport
        for (auto& [bus, frames] : bus_cmd) {
            bus->BlockingCycle(&frames[0], frames.size(), &replies);
            
            for (const auto& frame : replies) {
                servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
            }
        }   


        // Loan default-constructed memory directly from the shared pool
        auto sample_s = diagnostic_publisher.loan().value();
        // Loop through the Moteus servo data
        for (auto const& [id, result] : servo_data) {
            int target_idx = id - 1;
            if (target_idx >= 0 && target_idx < MOTOR_NUM) {
                // Access the array directly inside the shared memory segment using ->
                sample_s.payload_mut().motor_instance[target_idx] = motor_info::make_diag(result);
            }
        }
        // Send the sample off to any subscribers
        iox2::send(std::move(sample_s)).value();

        }
        
    // End safely
    for (auto& c : controllers) { c->SetStop(); }
    return 0;
}
