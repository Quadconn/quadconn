#include <iostream>

#include "quad_command.hpp"
#include "joint_angles.hpp"
#include "quad_config.hpp"
#include "quad_control.hpp"
#include "quad_ipc.hpp"
#include "gamepad_data.hpp"
#include "system_logic.hpp"

int main() {
    /* START: NODE DECLARATION */
    auto quadcontrol_node = make_node("quadcontrol_node");
    auto angle_publisher = make_publisher<BodyJointAngles>
        (make_service<BodyJointAngles>("BodyJointAngles", quadcontrol_node));
    auto controller_subscriber = make_subscriber<GamepadData>
        (make_service<GamepadData>("GamepadData", quadcontrol_node));
    auto interface_notifier = make_notifier
        (make_event("SystemLogic", quadcontrol_node));
    auto command_publisher = make_publisher<QuadCommand>
        (make_service<QuadCommand>("QuadCommand", quadcontrol_node));
    /* END: NODE DECLARATION */

    QuadControl quad;
    BodyJointAngles angles;
    QuadCommand command = {0.0};

    {
        bool notified = false;
        while (!notified && loop_waitms(100, quadcontrol_node)) {
            auto result = interface_notifier.notify_with_custom_event_id(
                iox2::EventId(iox2::bb::into<size_t>(SystemLogic::StartMotors)));
            
            if (result.has_value()) {
                notified = true;
            } else {
                std::cout << "Waiting for Motor Interface to be ready...\n";
            }
        }
    }
    int loop_count = 0;
    while (loop_waitms(quad::common::DT_MILLI, quadcontrol_node)) {
        
        // receiving joystick data
        auto received_val = ipc_receive(controller_subscriber);
        if (received_val.has_value()) {
            auto& data_ref = received_val.value();

            command.update(data_ref);
            std::cout << "horizontal_velocity: " << command.horizontal_velocity_x 
                      << "vertical_velocity: "   << command.horizontal_velocity_y 
                      << "\n"; 
            
        }

        quad.set_command(command);

        ipc_send_zerocopy(command_publisher, [&](auto& payload) {
            payload = command;   // copies all 8 fields
        });

        ipc_send_zerocopy(angle_publisher, [&](auto& payload) {payload = quad.step();});
        auto notify_res = interface_notifier.notify_with_custom_event_id(
            iox2::EventId(iox2::bb::into<size_t>(SystemLogic::QuadControlDone)));
        if (!notify_res.has_value()) {
            std::cerr << "Failed to notify QuadControlDone\n";
        }
        
        // TODO: upon conclusion of folding, send signal to end motor controller code
        if (quad._is_immobile) {
            auto notify_res = interface_notifier.notify_with_custom_event_id(
                iox2::EventId(iox2::bb::into<size_t>(SystemLogic::QuadControlDone)));
            if (!notify_res.has_value()) {
                std::cerr << "Failed to signal finished service\n";
            }
        }

    if (loop_count >= quad::common::MAX_LOOPS) {break;} else {loop_count++;}
    }

    return 0;
}
