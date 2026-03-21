#include <iostream>

#include "quad_command.hpp"
#include "joint_angles.hpp"
#include "quad_config.hpp"
#include "quad_control.hpp"
#include "quad_ipc.hpp"
#include "gamepad_data.hpp"


int main() {
    /* START: NODE DECLARATION */
    auto quadcontrol_node = make_node("quadcontrol_node");
    auto angle_publisher = make_publisher<BodyJointAngles>
        (make_service<BodyJointAngles>("BodyJointAngles", quadcontrol_node));
    auto controller_subscriber = make_subscriber<GamepadData>
        (make_service<GamepadData>("GamepadData", quadcontrol_node));
    /* END: NODE DECLARATION */

    QuadControl quad;
    BodyJointAngles angles;
    QuadCommand command;

    while (loop_waitms(quad::common::DT_MILLI, quadcontrol_node)) {
        
        // receiving joystick data
        auto received_val = ipc_receive(controller_subscriber);
        if (received_val.has_value()) {
            auto& data_ref = received_val.value();

            command.update(data_ref);
            // TODO DR: When this becomes dynamic handle it better, for now just always
            // using this height
            command.height = -(quad::config::L1 + (quad::config::L2 / 2));
        }

        quad.set_command(command);

        std::cout << "Sending (Vx, Vy, Yaw): (" 
                  << command.horizontal_velocity_x << ", "
                  << command.horizontal_velocity_y << ", "
                  << command.yaw_rate << ")" << std::endl;

        ipc_send_zerocopy(angle_publisher, [&](auto& payload) {payload = quad.step_gait();});
    }

    return 0;
}
