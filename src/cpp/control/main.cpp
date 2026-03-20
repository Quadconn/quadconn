#include <iostream>

#include "command.hpp"
#include "joint_angles.hpp"
#include "quad_config.hpp"
#include "quad_control.hpp"
#include "quad_ipc.hpp"
#include "gamepad_data.hpp"

#define HORIZONTAL_MAX 0.4
#define VERTICAL_MAX 0.3

inline double deadzone(double input_joystick) {
    // hardcoded deadzone of 0.05 to prevent jitter
    return (fabs(input_joystick) < 0.05f ? 
        0.0f : 
        input_joystick);
}


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
    Command command = {
        .horizontal_velocity_x = 0.4, //  [-1,1] - => down, + => up
        .horizontal_velocity_y = -0.3, // [-1,1] - => right, + => left
        .yaw_rate = 0.0,
        .height = -(quad::config::L1 + (quad::config::L2 / 2))
    };

    while (loop_waitms(quad::config::dt_milli, quadcontrol_node)) {
        
        // receiving joystick data
        auto received_val = ipc_receive(controller_subscriber);
        if (received_val.has_value()) {
            auto& data_ref = received_val.value();

            // assigning velocities according to controller values
            command.horizontal_velocity_x = HORIZONTAL_MAX * deadzone(data_ref.ly);
            command.horizontal_velocity_y = -VERTICAL_MAX  * deadzone(data_ref.lx);
        }

        quad.set_command(command);

        std::cout << "Sending: " << "horizontal velocity X: " << command.horizontal_velocity_x 
         << "horizontal velocity Y: " << command.horizontal_velocity_y << std::endl;

        ipc_send_zerocopy(angle_publisher, [&](auto& payload) {payload = quad.step_gait();});
    }

    return 0;
}
