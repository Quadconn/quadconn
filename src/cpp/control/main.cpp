#include <iostream>
#include <cmath>

#include <iox2/iceoryx2.hpp>

#include "command.hpp"
#include "joint_angles.hpp"
#include "quad_config.hpp"
#include "quad_control.hpp"
#include "quad_ipc.hpp"


int main() {

    JointAngles leftLegAngles;

    QuadIpcPublisher<JointAngles> ipc("Control", "joint_angles");
    QuadControl quad;

    Command command = {
        .horizontal_velocity_x = 0.4,
        .horizontal_velocity_y = -0.3,
        .yaw_rate = 0.0,
        .height = -(QuadConfig::L1 + (QuadConfig::L2 / 2))
    };

    quad.set_command(command);

    while (ipc.wait(10)) {

        ipc.send(quad.step_gait());

        std::cout << "Sent!" << std::endl;
    }


    return 0;
}
