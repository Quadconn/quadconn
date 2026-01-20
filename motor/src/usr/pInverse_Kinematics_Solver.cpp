#include "iox2/iceoryx2.hpp"
#include "three_dof_theta.hpp"
#include "xyz.hpp"

int main(int argc, char** argv) {
    using namespace iox2;

    // update depending on values sent from 3D inverse kinematics
    constexpr bb::Duration UPDATE_RATE = bb::Duration::from_millis(750);
    // value sent to publisher node
    three_dof_theta ik_theta_val{};

    /* START: BRACKET GUARD -- INIT NODE */
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();
    auto p_service = node.service_builder(ServiceName::create("three_dof_theta").value())
                    .publish_subscribe<three_dof_theta>()
                    .open_or_create()
                    .value();
    auto publisher = p_service.publisher_builder().create().value();
    
    auto s_service = node.service_builder(ServiceName::create("target_coords").value())
                    .publish_subscribe<target_coords>()
                    .open_or_create()
                    .value();
    auto subscriber = s_service.subscriber_builder().create().value();
    /* END: BRACKET GUARD -- INIT NODE */

    // for now, can populate with non-specific vector values to send 
    std::vector<std::vector<double>> trajectory = {
        {M_PI,          -M_PI,         2 * M_PI},
        {4 * M_PI,      0.0,         4 * -M_PI},
        {0.0,          9.0 * M_PI,         9.0 * M_PI},
        {9.0 * -M_PI,          0.0,         9.0 * -M_PI},
        {0.0,          9.0 * -M_PI,         9.0 * M_PI}};

    // 
    int i = 0;
    while(node.wait(UPDATE_RATE).has_value() && (i < trajectory.size())) {
        
        /* START: USER CODE: FILL three_dof_theta w/ angles to send to sMotor_Interface */ 
        ik_theta_val.theta1 = trajectory[i][0];
        ik_theta_val.theta2 = trajectory[i][1];
        ik_theta_val.theta3 = trajectory[i][2];
        ++i;
        /* END: USER CODE */ 
        
        /* END: BRACKET GUARD -- SEND PUB VALUE */
        auto sample = publisher.loan_uninit().value();
        auto initialized_sample = sample.write_payload(ik_theta_val);
        std::cout << "sending values " << i << ":" << ik_theta_val << std::endl;
        send(std::move(initialized_sample)).value();
        /* END: BRACKET GUARD -- SEND PUB VALUE */
    }
    return 0;
}