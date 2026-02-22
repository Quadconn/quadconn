#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>

#include "moteus.h"
#include "../common/joint_angles.hpp"
#include "motor_diagnostics.hpp"
#include "iox2/iceoryx2.hpp"
#include "../common/quad_ipc.hpp"

#define UPDATE_RATE_MS 5
#define MOTOR_NUM 3

// Convert radians to turns
inline double rad2turns(double radians) {
    return 9*(radians / (2.0 * M_PI));
}

int main(int argc, char** argv) {
    using namespace mjbots;
    using namespace iox2;


    
    // change usb-id depending on motor id used, map motor controller node ids to bus
    const auto bus_a = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00");
    const auto bus_b = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00");
    std::map<int, std::shared_ptr<moteus::Fdcanusb>> id_to_bus = {
        {1, std::move(bus_a)},
        {2, std::move(bus_a)},
        {3, std::move(bus_a)}
        // {4, std::move(bus_b)},
        // {5, std::move(bus_b)},
        // {6, std::move(bus_b)}
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
    std::array<double, MOTOR_NUM> target_angles{0.0, 0.0, 0.0};
    std::map<int, moteus::Query::Result> servo_data;
    std::map<std::shared_ptr<moteus::Fdcanusb>, std::vector<moteus::CanFdFrame>> bus_cmd;
    moteus::PositionMode::Command cmd;
    // Execution loop
    std::cout << "starting motor loop\n";
    
    // attempt to receive value on loop duration
    while (loop_waitms(UPDATE_RATE_MS, motor_node)) { 
        
        // clear on new loop
        servo_data.clear();
        bus_cmd.clear();


        // receiving Sample 
        auto sample_r = angle_subscriber.receive().value();
        if (sample_r.has_value()) {
            // reference to sample payload
            const auto& payload = sample_r.value().payload();
    
            for (size_t i = 0; i < controllers.size(); ++i) {
                float target = (i == 0) ? payload.hip_roll  :
                               (i == 1) ? payload.hip_pitch :
                                          payload.knee_pitch;
                cmd.position = rad2turns(target_angles[i]);
                cmd.velocity = std::numeric_limits<double>::quiet_NaN(); 
                auto frame = controllers[i]->MakePosition(cmd);
                // first holds value of transport path, second holds value of command frame
                bus_cmd[id_to_bus[controllers[i]->options().id]].push_back(frame);
            }
        }

        std::vector<moteus::CanFdFrame> replies;
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
            if (target_idx >= 0 && target_idx < MOTOR_COUNT) {
                // Access the array directly inside the shared memory segment using ->
                sample_s.payload_mut().motor_instance[target_idx] = motor_info::make_diag(result);
            }
        }
        // Send the sample off to any subscribers
        iox2::send(std::move(sample_s));
                    
        }
        
    // End safely
    for (auto& c : controllers) { c->SetStop(); }
    return 0;
}