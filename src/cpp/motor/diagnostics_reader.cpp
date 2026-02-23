#include "iox2/iceoryx2.hpp"
#include "../common/gamepad_data.hpp"
#include "../common/quad_ipc.hpp"
#include "../common/joint_angles.hpp"

#define SAMPLE_RATE 50
int main(int argc, char** argv) {
    using namespace iox2;

    constexpr bb::Duration UPDATE_RATE = bb::Duration::from_millis(1);
    /* START: BRACKET GUARD -- INIT NODE */
    auto node = make_node("diagnostics");


    auto angle_subscriber = make_subscriber<JointAngles>
           (make_service<JointAngles>("JointAngles", node));
    // auto game_subscriber = make_subscriber<GamepadData>
    //        (make_service<GamepadData>("GamepadData", node));
    /* END: BRACKET GUARD -- INIT NODE */

    while(loop_waitms(SAMPLE_RATE, node)) {
        
        
        // receiving joystick data
        auto received_val = ipc_receive(angle_subscriber);
        // auto received_val2 = ipc_receive(game_subscriber);
        if (received_val.has_value()) {
            // reference to value 
            auto& data_ref = received_val.value();
            // auto& other_data = received_val2.value();
            // assigning velocities according to controller values
            std::cout << data_ref << std::endl;
            // td::cout << other_data << std::endl;
        }
   
    }
    return 0;
}