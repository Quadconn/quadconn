#include <iostream>

#include "command.hpp"
#include "joint_angles.hpp"
#include "quad_config.hpp"
#include "quad_control.hpp"
#include "quad_ipc.hpp"


int main() {

    QuadIpcPublisher<BodyJointAngles> ipc("Control", "joint_angles");
    QuadControl quad;

    // TODO DR: Integrate remote controller to supply these commands instead of
    // a static one here
    Command command = {
        .horizontal_velocity_x = 0.4,
        .horizontal_velocity_y = -0.3,
        .yaw_rate = 0.0,
        .height = -(quad::config::L1 + (quad::config::L2 / 2))
    };

    quad.set_command(command);

    BodyJointAngles angles;
    while (ipc.wait(quad::config::dt_milli)) {

        angles = quad.step_gait();

        ipc.send(angles);
    }

    return 0;
}
