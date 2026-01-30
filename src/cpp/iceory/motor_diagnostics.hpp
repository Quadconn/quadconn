#include <cstdint>
#include <iostream>


struct motor_diagnostics {
    int mode; int fault; int trajectory_complete;
    double position; double velocity; double torque;
    double q_current; double d_current; double abs_position;
    double power; double motor_temperature; double voltage;
    double temperature;
    static constexpr const char* IOX2_TYPE_NAME = "motor_diagnostics";
};


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