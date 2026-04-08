#include "iox2/iceoryx2.hpp"
#include "gamepad_data.hpp"
#include "quad_ipc.hpp"
#include "joint_angles.hpp"
#include "system_logic.hpp"
#include "motor_diagnostics.hpp"
#include <fstream>
#include <iomanip>
#include <csignal> // Required for catching signals like Ctrl+C
#define SAMPLE_RATE_MS 1000

volatile sig_atomic_t keep_running = 1;

void signal_handler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Safely shutting down..." << std::endl;
    // When Ctrl+C is pressed, change the flag to 0
    keep_running = 0; 
}

int main(int argc, char** argv) {
    using namespace iox2;

    std::signal(SIGINT, signal_handler);

    /* START: BRACKET GUARD -- INIT NODE */
    auto node = make_node("diagnostics");


    auto angle_subscriber = make_subscriber<BodyJointAngles>
           (make_service<BodyJointAngles>("BodyJointAngles", node));
    // auto system_listener = make_listener(make_event("SystemLogic", node));
    auto diagnostics_subscriber = make_subscriber<MotorDiagnosticsArray>
            (make_service<MotorDiagnosticsArray>("MotorDiagnosticsArray", node));
    

    // global variables
    
    MotorDiagnosticsArray init_array;

    // create csv file
    std::ofstream csvFile("data_log.csv", std::ios::out);
    // Always check if the file opened successfully
    if (!csvFile.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return 1;
    }
    csvFile << std::fixed << std::setprecision(4);
    std::string motor_power;
    std::string motor_torque;
    std::string motor_velocity;
    for (int i = 1; i < (MOTOR_COUNT+1); i++) {
        motor_power += "Motor " + std::to_string(i) + " Power,";
        motor_torque += "Motor " + std::to_string(i) + " Torque,";
        motor_velocity += "Motor " + std::to_string(i) + " Velocity,";
    }
    csvFile << "Timestamp," 
            << motor_power << motor_torque << motor_velocity 
            << "Battery Voltage\n";

    int loop_count = 1;
    while(loop_waitms(SAMPLE_RATE_MS, node) && keep_running) {
        double voltage = 0.0;
        std::ostringstream power_ss, torque_ss, velocity_ss;
            power_ss << std::fixed << std::setprecision(4);
            torque_ss << std::fixed << std::setprecision(4);
            velocity_ss << std::fixed << std::setprecision(4);

        csvFile << loop_count << ',';
        MotorDiagnosticsArray read_val = ipc_receive(diagnostics_subscriber).value_or(init_array);
        
        for (int i = 0; i < MOTOR_COUNT; i++) {
            voltage += read_val.motor_instance[i].voltage;
            power_ss << read_val.motor_instance[i].power << ',';
            torque_ss << read_val.motor_instance[i].torque << ',';
            velocity_ss << read_val.motor_instance[i].velocity << ',';
        }
        csvFile << power_ss.str() << torque_ss.str() << velocity_ss.str()
                << (voltage/static_cast<double>(MOTOR_COUNT)) << '\n';
        std::cout << "battery voltage: " << (voltage/static_cast<double>(MOTOR_COUNT)) << '\n';
        loop_count++;
    }
    csvFile.close();
    std::cout << "closed succesfully \n";
    return 0;
}


        // auto event = system_listener.try_wait_one();
        // if(event.has_value()) {
        //     auto event_val = event.value();
        //     if (event_val.has_value()) {
        //         if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::KillMotors) {
        //                 std::cout << "stopping diagnostics loop";
        //                 break;
        //         }
        //         if(bb::into<SystemLogic>(event.value()->as_value()) == SystemLogic::StartMotors) {
        //                 std::cout << "idk u pressed start or something";
        //         }                
        //     }
        // }