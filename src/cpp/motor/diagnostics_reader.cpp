#include "iox2/iceoryx2.hpp"
#include "gamepad_data.hpp"
#include "quad_ipc.hpp"
#include "joint_angles.hpp"

#define SAMPLE_RATE_MS 50
int main(int argc, char** argv) {
    using namespace iox2;

    constexpr bb::Duration UPDATE_RATE = bb::Duration::from_secs(1);
    /* START: BRACKET GUARD -- INIT NODE */
    auto node = make_node("diagnostics");


    auto angle_subscriber = make_subscriber<JointAngles>
           (make_service<JointAngles>("JointAngles", node));
    // auto game_subscriber = make_subscriber<GamepadData>
    //        (make_service<GamepadData>("GamepadData", node));
    /* END: BRACKET GUARD -- INIT NODE */

    while(loop_waitms(SAMPLE_RATE_MS, node)) {
        
        
        // receiving joystick data
        auto received_val = ipc_receive(angle_subscriber);
        if (received_val.has_value()) {
            // pull value from IPC_Receive
            auto& data_ref = received_val.value();
            std::cout << data_ref << std::endl;
        }
   
    }
    return 0;
}