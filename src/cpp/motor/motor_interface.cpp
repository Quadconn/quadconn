#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>
#include <filesystem>
#include <stdexcept>
#include <systemd/sd-daemon.h>

#include "joint_angles.hpp"
#include "motor_diagnostics.hpp"
#include "quad_ipc.hpp"
#include "motor_config.hpp"
#include "system_logic.hpp"
#include "iox2/iceoryx2.hpp"

#define ACCEL_LIMIT 2000
#define KP_SCALE    1.0

// Helper to generate random double in a range
double f_rand(double min, double max) {
    return min + (double)rand() / RAND_MAX * (max - min);
}

int main(int argc, char** argv) {
    using namespace iox2;
    srand(time(NULL));

    /* START: BRACKET GUARD -- Init Node */
    auto motor_node = make_node("motor_node");

    auto diagnostic_publisher = make_publisher<MotorDiagnosticsArray>
        (make_service<MotorDiagnosticsArray>("MotorDiagnosticsArray", motor_node));
    auto angle_subscriber = make_subscriber<BodyJointAngles>
        (make_service<BodyJointAngles>("BodyJointAngles", motor_node));
    auto system_listener = make_listener
        (make_event("SystemLogic", motor_node));

    /* END: BRACKET GUARD -- Init Node */


    // after motor has finished initializing, start control service
    // and pause until angles are being published
    std::cout << "waiting for control code\n";
    sd_notify(1, "READY=1\n"
                "STATUS=Finished Initialization...");
    while (true) {
        auto event = system_listener.blocking_wait_one();
        if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::StartMotors) {
            std::cout << "starting motor_controller process\n";
            break;
        }
    }

    // Prepare batch of commands
    BodyJointAngles target_val{};


    // Execution loop
    std::cout << "starting motor loop\n";
    // attempt to receive value, will wait until events occur
    while (true) { 
        
        auto event = system_listener.blocking_wait_one();
        // catch stop signal
        if (event.has_value() && event.value().has_value()) {
            // supplied upon closing of control code
            if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::KillMotors) {
                std::cout << "stopping motor_controller process";
                break;
            } else
            if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::QuadControlDone) {
                        // receiving values from JointAngles struct
                target_val = ipc_receive(angle_subscriber).value_or(target_val);
                std::cout << "angle values received" << target_val << "\n";
                
                
                // Loan default-constructed memory directly from the shared pool
                auto sample_s_opt = diagnostic_publisher.loan();
                if (sample_s_opt.has_value()) {
                    auto sample_s = std::move(sample_s_opt.value());
                    // Send the sample off to any subscribers
                    auto& array_payload = sample_s.payload_mut();
                    for (int i = 0; i < MOTOR_COUNT; ++i) {
                        auto& m = array_payload.motor_instance[i];
                        
                        // Fill with dummy data
                        m.mode = rand() % 16;
                        m.trajectory_complete = 1;
                        m.position = f_rand(-3.14, 3.14);
                        m.velocity = f_rand(-10.0, 10.0);
                        m.torque = f_rand(-2.0, 2.0);
                        m.voltage = f_rand(22.0, 26.0);
                        m.temperature = f_rand(30.0, 65.0);
                        m.motor_temperature = m.temperature + 5.0;
                    }
                    if (!iox2::send(std::move(sample_s)).has_value()) {
                        std::cerr << "Failed to send diagnostic sample!" << std::endl;
                    }
                }
            } 
        }
    }

    return 0;
}
