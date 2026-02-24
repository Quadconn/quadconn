#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>

#include "moteus.h"
#include "../common/joint_angles.hpp"
#include "../common/motor_diagnostics.hpp"
#include "iox2/iceoryx2.hpp"
#include "../common/quad_ipc.hpp"

#define UPDATE_RATE_MS 5
#define MOTOR_NUM  3
#define GEAR_RATIO 9

// Convert radians to turns
inline double rad2turns(double radians) {
    return GEAR_RATIO*(radians / (2.0 * M_PI));
}

int main(int argc, char** argv) {
    using namespace mjbots;
    using namespace iox2;

    
    
    // change usb-id depending on motor id used, map motor controller node ids to bus
    // const auto bus_a = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00");
    const auto bus_b = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00");
    std::map<int, std::shared_ptr<moteus::Fdcanusb>> id_to_bus = {
        {4, bus_b},
        {5, bus_b},
        {6, bus_b}
        // {4, bus_b},
        // {5, bus_b},
        // {6, bus_b}
    };

    /* START: BRACKET GUARD -- Init Node */
    auto motor_node = make_node("motor_node");

    auto diagnostic_publisher = make_publisher<MotorDiagnosticsArray>
        (make_service<MotorDiagnosticsArray>("MotorDiagnosticsArray", motor_node));
    auto angle_subscriber = make_subscriber<JointAngles>
        (make_service<JointAngles>("JointAngles", motor_node));

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
    JointAngles target_val = {.hip_roll=0.0, .hip_pitch=0.0, .knee_pitch=0.0};

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
        auto sample_r = angle_subscriber.receive().value();
        if (sample_r.has_value()) {
            target_val.hip_roll =   sample_r.value().payload().hip_roll;
            target_val.hip_pitch =  sample_r.value().payload().hip_pitch;
            target_val.knee_pitch = sample_r.value().payload().knee_pitch;
        }

        //  build the CAN frames using the latest targets
        for (size_t i = 0; i < controllers.size(); ++i) {
            double angle_cmd = (i == 0) ? target_val.hip_roll :
                               (i == 1) ? target_val.hip_pitch :
                                          target_val.knee_pitch;

            cmd.position = rad2turns(angle_cmd);
            cmd.velocity = std::numeric_limits<double>::quiet_NaN(); 
            auto frame = controllers[i]->MakePosition(cmd);
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