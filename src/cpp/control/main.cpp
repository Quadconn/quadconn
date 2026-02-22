#include <iostream>

#include "command.hpp"
#include "joint_angles.hpp"
#include "quad_config.hpp"
#include "quad_control.hpp"
#include "quad_ipc.hpp"



int main() {
    

    JointAngles leftLegAngles;
    auto quadcontrol_node = make_node("quadcontrol_node");
    auto angle_publisher = make_publisher<JointAngles>(make_service<JointAngles>("joint_angles", quadcontrol_node));
    
    QuadControl quad;

    // TODO DR: Integrate remote controller to supply these commands instead of
    // a static one here

    // scale from [-1 1]
    // 
    // left stick modulates x-y Forward -> increase vel_x Down -> decrease vel_x
    // Left -> increase vel_y, Right -> decrase vel_y


    // 
    Command command = {
        .horizontal_velocity_x = 0.4, //  [-1,1] - => down, + => up
        .horizontal_velocity_y = -0.3, // [-1,1] - => right, + => left
        .yaw_rate = 0.0,
        .height = -(QuadConfig::L1 + (QuadConfig::L2 / 2))
    };

    quad.set_command(command);
    JointAngles angles;

    
    while (loop_waitms(QuadConfig::dt_milli, quadcontrol_node)) {

        angles = quad.step_gait();

        ipc_send(angles, angle_publisher);

        std::cout << "Sent: " << angles << std::endl;
    }

    return 0;
}
