#pragma once
#include <cstdint>
#include <iostream>
#include "moteus.h"
#define MOTOR_COUNT 12
#define GEAR_RATIO 9


struct MotorDiagnostics {
    // mode diagnostics
    int mode; int fault; int trajectory_complete;
    // pvt 
    double position; double velocity; double torque;
    // current values
    double q_current; double d_current; double abs_position;
    // power diagnostics
    double power; double motor_temperature; double voltage; double temperature;
    static constexpr const char* IOX2_TYPE_NAME = "MotorDiagnostics";
};

struct MotorDiagnosticsArray {
    MotorDiagnostics motor_instance[MOTOR_COUNT];
    static constexpr const char* IOX2_TYPE_NAME = "MotorDiagnosticsArray";
};



inline auto operator<<(std::ostream& stream, const MotorDiagnostics& value) -> std::ostream& {
    stream << "MotorDiagnostics { "
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

inline double rad2turns(double radians) {
    return GEAR_RATIO*(radians / (2.0 * M_PI));
}

namespace motor_info {
    
    inline void populate_diag(const mjbots::moteus::Query::Result& result, MotorDiagnostics& out_diag) {
    // Write directly into 'out_diag' (which lives in shared memory)
    out_diag.mode = static_cast<int>(result.mode);
    out_diag.fault = result.fault;
    out_diag.trajectory_complete = result.trajectory_complete;
    out_diag.position = result.position;
    out_diag.velocity = result.velocity;
    out_diag.torque = result.torque;
    out_diag.q_current = result.q_current;
    out_diag.d_current = result.d_current;
    out_diag.abs_position = result.abs_position;
    out_diag.power = result.power;
    out_diag.motor_temperature = result.motor_temperature;
    out_diag.voltage = result.voltage;
    out_diag.temperature = result.temperature;
    }

    inline std::string_view mode_to_string(int mode) {
        switch(mode) {
            case 0:  return "kStopped";
            case 1:  return "kFault";
            case 2:  return "kEnabling";
            case 3:  return "kCalibrating";
            case 4:  return "kCalibrationComplete";
            case 5:  return "kPwm";
            case 6:  return "kVoltage";
            case 7:  return "kVoltageFoc";
            case 8:  return "kVoltageDq";
            case 9:  return "kCurrent";
            case 10: return "kPosition";
            case 11: return "kPositionTimeout";
            case 12: return "kZeroVelocity";
            case 13: return "kStayWithin";
            case 14: return "kMeasureInd";
            case 15: return "kBrake";
            default: return "UNKNOWN";
        }
    }

    inline std::string_view fault_to_string(int fault) {
        switch(fault) {
            case 32:  return "calibration fault";
            case 33:  return "motor driver fault";
            case 34:  return "over voltage";
            case 35:  return "encoder fault";
            case 36:  return "motor not configured";
            case 37:  return "pwm cycle overrun";
            case 38:  return "over temperature";
            case 39:  return "outside limit";
            case 40:  return "under voltage";
            case 41:  return "config changed";
            case 42:  return "theta invalid";
            case 43:  return "position invalid";
            case 44:  return "driver enable fault";
            case 45:  return "stop position deprecated";
            case 46:  return "timing violation";
            case 47:  return "bemf feedforward no accel";
            case 48:  return "invalid limits";
            case 96:  return "servo.max_velocity";
            case 97:  return "servo.max_power_W";
            case 98:  return "the maximum system voltage";
            case 99:  return "servo.max_current_A";
            case 100: return "servo.fault_temperature";
            case 101: return "servo.motor_fault_temperature";
            case 102: return "the commanded maximum torque";
            case 103: return "servopos.position_min or servopos.position_max";
            default:  return "UNKNOWN";
        }
    }
}


