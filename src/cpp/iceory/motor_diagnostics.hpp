#include <cstdint>
#include <iostream>
#include "moteus.h"
#define MOTOR_COUNT 12

struct motor_diagnostics {
    // mode diagnostics
    int mode; int fault; int trajectory_complete;
    // pvt 
    double position; double velocity; double torque;
    // current values
    double q_current; double d_current; double abs_position;
    // power diagnostics
    double power; double motor_temperature; double voltage; double temperature;
    static constexpr const char* IOX2_TYPE_NAME = "motor_diagnostics";
};

struct motor_diagnostics_array {
    motor_diagnostics motor_d[MOTOR_COUNT];
    static constexpr const char* IOX2_TYPE_NAME = "motor_diagnostics_array";
};

inline motor_diagnostics make_diag(const mjbots::moteus::Query::Result& r) {
    return {static_cast<int>(r.mode), r.fault, r.trajectory_complete,
            r.position, r.velocity, r.torque,
            r.q_current, r.d_current, r.abs_position,
            r.power, r.motor_temperature, r.voltage, r.temperature};
}

inline auto operator<<(std::ostream& stream, const motor_diagnostics& value) -> std::ostream& {
    stream << "motor_diagnostics { "
           << "mode: " << value.mode
           << ", fault: " << value.fault
           << ", trajectory_complete: " << value.trajectory_complete
           << ", position: " << value.position
           << ", velocity: " << value.velocity
           << ", torque: " << value.torque
           << ", q_current: " << value.q_current
           << ", d_current: " << value.d_current
           << ", abs_position: " << value.abs_position
           << ", power: " << value.power
           << ", motor_temperature: " << value.motor_temperature
           << ", voltage: " << value.voltage
           << ", temperature: " << value.temperature
           << " }";
    return stream;
}

