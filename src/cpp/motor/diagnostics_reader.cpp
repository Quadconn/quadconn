#include "iox2/iceoryx2.hpp"
#include "gamepad_data.hpp"
#include "quad_ipc.hpp"
#include "joint_angles.hpp"
#include "system_logic.hpp"
#define SAMPLE_RATE_MS 50

int main(int argc, char** argv) {
    using namespace iox2;


    /* START: BRACKET GUARD -- INIT NODE */
    auto node = make_node("diagnostics");


    auto angle_subscriber = make_subscriber<BodyJointAngles>
           (make_service<BodyJointAngles>("BodyJointAngles", node));
    auto system_listener = make_listener(make_event("SystemLogic", node));
    
    while(loop_waitms(SAMPLE_RATE_MS, node)) {
  
        auto event = system_listener.try_wait_one();
        // catch stop signal
        if(event.has_value()) {
            auto event_val = event.value();
            if (event_val.has_value()) {
                if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::KillMotors) {
                        std::cout << "stopping diagnostics loop";
                        break;
                }
                if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::StartMotors) {
                        std::cout << "idk u pressed start or something";
                }                
            }
        }
    }

    return 0;
}