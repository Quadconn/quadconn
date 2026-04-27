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

    interface_notifier.notify_with_custom_event_id(iox2::EventId(
                    iox2::bb::into<size_t>(SystemLogic::StartMotors))).value();

    while (loop_waitms(quad::common::DT_MILLI, quadcontrol_node)) {
        
        // receiving joystick data
        auto received_val = ipc_receive(controller_subscriber);
        if (received_val.has_value()) {
            auto& data_ref = received_val.value();

            command.update(data_ref);
        }

        quad.set_command(command);

        ipc_send_zerocopy(command_publisher, [&](auto& payload) {
            payload = command;   // copies all 8 fields
        });

        ipc_send_zerocopy(angle_publisher, [&](auto& payload) {payload = quad.step();});
        interface_notifier.notify_with_custom_event_id(iox2::EventId(
                           iox2::bb::into<size_t>(SystemLogic::QuadControlDone))).value();
    }

    return 0;
}
