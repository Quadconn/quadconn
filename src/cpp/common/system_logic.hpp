#pragma once

#include <cstdint>
#include <stddef.h>
#include "iox2/iceoryx2.hpp"


enum class SystemLogic : uint8_t {
    GamepadRunning = 0,
    QuadControlRunning = 1,
    MotorsRunning = 2,
    GUIRunning = 3,
    // control code 
    StartMotors = 4,
    KillMotors = 5,
    QuadControlDone = 6,
    Unknown = 67
};

namespace iox2 {
namespace bb {
template <>
constexpr auto from<SystemLogic, size_t>(const SystemLogic value) noexcept -> size_t {
    return static_cast<uint8_t>(value);
}

template <>
constexpr auto from<size_t, SystemLogic>(const size_t value) noexcept -> SystemLogic {
    switch (value) {
    case into<size_t>(SystemLogic::GamepadRunning):
        return SystemLogic::GamepadRunning;
    case into<size_t>(SystemLogic::QuadControlRunning):
        return SystemLogic::QuadControlRunning;
    case into<size_t>(SystemLogic::MotorsRunning):
        return SystemLogic::MotorsRunning;
    case into<size_t>(SystemLogic::GUIRunning):
        return SystemLogic::GUIRunning;
    case into<size_t>(SystemLogic::StartMotors):
        return SystemLogic::StartMotors;
    case into<size_t>(SystemLogic::KillMotors):
        return SystemLogic::KillMotors;    
    case into<size_t>(SystemLogic::QuadControlDone):
        return SystemLogic::QuadControlDone;
    default:
        return SystemLogic::Unknown;
    }
    IOX2_UNREACHABLE();
}
} // namespace bb
} // namespace iox2

