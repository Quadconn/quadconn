import iceoryx2 as iox2
from enum import IntEnum

# helpers for decoding gamepad button presses

class SystemLogic(IntEnum):
    # dictates if individual modules are running
    GamepadRunning = 0
    QuadControlRunning = 1
    MotorsRunning = 2
    GUIRunning = 3
    # control code 
    StartMotors = 4
    KillMotors = 5
    Unknown = 67
    

def to_event_id(event: SystemLogic) -> iox2.EventId:
    """Converts enum value to iceoryx2 EventId."""
    return iox2.EventId.new(int(event))

def from_event_id(event_id: iox2.EventId) -> SystemLogic:
    """Converts iceoryx2 EventId to enum value."""
    value = event_id.as_value
    for event in SystemLogic:
        if int(event) == value:
            return event
    return SystemLogic.Unknown