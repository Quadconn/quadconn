#include <iostream>

#include "command.hpp"
#include "joint_angles.hpp"
#include "quad_config.hpp"
#include "quad_control.hpp"
#include "quad_ipc.hpp"
#include "../common/gamepad_data.hpp"

#define HORIZONTAL_MAX 0.4
#define VERTICAL_MAX 0.3

inline double deadzone(double input_joystick) {
    // hardcoded deadzone of 0.05 to prevent jitter
    return (fabs(input_joystick) < 0.05f ? 
        0.0f : 
        floor((input_joystick*100+0.5)/100));
}


int main() {
    JointAngles leftLegAngles;

    /* START: NODE DECLARATION */
    auto quadcontrol_node = make_node("quadcontrol_node");
    auto angle_publisher = make_publisher<JointAngles>
        (make_service<JointAngles>("JointAngles", quadcontrol_node));
    auto controller_subscriber = make_subscriber<GamepadData>
        (make_service<GamepadData>("GamepadData", quadcontrol_node));
    /* END: NODE DECLARATION */
    QuadControl quad;


    Command command = {
        .horizontal_velocity_x = 0.4, //  [-1,1] - => down, + => up
        .horizontal_velocity_y = -0.3, // [-1,1] - => right, + => left
        .yaw_rate = 0.0,
        .height = -(QuadConfig::L1 + (QuadConfig::L2 / 2))
    };

    JointAngles angles;

    
    while (loop_waitms(QuadConfig::dt_milli, quadcontrol_node)) {
        
        // receiving joystick data
        auto received_val = ipc_receive(controller_subscriber);
        if (received_val.has_value()) {
            auto& data_ref = received_val.value();

            // assigning velocities according to controller values
            command.horizontal_velocity_x = HORIZONTAL_MAX * deadzone(data_ref.ly);
            command.horizontal_velocity_y = -VERTICAL_MAX  * deadzone(data_ref.lx);
        }



        quad.set_command(command);

        // angles = quad.step_gait();
        std::cout << "Sending: " << "horizontal velocity X: " << command.horizontal_velocity_x 
         << "horizontal velocity Y: " << command.horizontal_velocity_y << std::endl;

        ipc_send_zerocopy(angle_publisher, [&](auto& payload) {payload = quad.step_gait();});


    }

    return 0;
}
