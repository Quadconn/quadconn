#include <iostream>
#include <cmath>

#include "iox2/iceoryx2.hpp"

#include "leg_ik.hpp"


constexpr double ABDUCTION_OFFSET = 0.04241 + 0.069;
constexpr double L1  = 0.19425;
constexpr double L2  = 0.140;

int main() {

    JointAngles leftLeg;
    Point target;
    Point result;

    target.x = L1 + L2;
    target.y = ABDUCTION_OFFSET;
    target.z = 0.0;

    std::cout << "Target: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
    if (leg_ik(leftLeg, target.x, target.y, target.z)) {

        leg_fk(result, leftLeg.hip_roll, leftLeg.hip_pitch, leftLeg.knee_pitch);
        std::cout << "Result: (" << result.x << ", " << result.y << ", " << result.z << ")" << std::endl;

        double error = std::sqrt(sq(target.x - result.x) + sq(target.y - result.y) + sq(target.z - result.z));

        if (error < 0.005) {
            std::cout << "Success!" << std::endl;
        } else {
            std::cout << "Fail!" << std::endl;
        }
        std::cout << "Error = " << error << std::endl;
    }

    // ----- Iceory --------

    // Top level node facilitating communication
    auto node = iox2::NodeBuilder()
        .name(iox2::NodeName::create("Control").value())
        .create<iox2::ServiceType::Ipc>().value();

    // Service that can be published/subscribed to
    auto service = node.service_builder(iox2::ServiceName::create("joint_angles").value())
        .publish_subscribe<JointAngles>()
        .open_or_create()
        .value();

    // Publisher to the service
    auto publisher = service.publisher_builder().create().value();

    while (node.wait(iox2::bb::Duration::from_millis(500)).has_value()) {
        // Uninitialized sample of publishers memory pool
        auto sample = publisher.loan_uninit().value();

        // Initialize the sample to have valid data
        auto initialized_sample = sample.write_payload(JointAngles {.hip_roll = leftLeg.hip_roll, 
                                                                    .hip_pitch = leftLeg.hip_pitch, 
                                                                    .knee_pitch = leftLeg.knee_pitch});
        // Actually send the value to the service
        iox2::send(std::move(initialized_sample)).value();

        std::cout << "Sent!" << std::endl;
    }

    return 0;
}
