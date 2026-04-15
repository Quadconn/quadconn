#include "quad_control.hpp"

#include <cassert>

#include <cstddef>
#include <cmath>
#include <algorithm>
#include <numbers>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "quad_common.hpp"
#include "quad_config.hpp"
#include "joint_angles.hpp"
#include "quad_command.hpp"

namespace config = quad::config;
namespace common = quad::common;

// Helper

inline double sq(double a) {
    return a*a;
}

// Public Methods

void QuadControl::set_command(const QuadCommand& command) {
    _command = command;

    double z_clearance = abs(_height + _command.height_rate);
    if ((config::z_clearance_min < z_clearance) && (z_clearance < config::z_clearance_max)) {
        _height += _command.height_rate;
    }

}


BodyJointAngles QuadControl::step() {
    // TODO DR: See if its better to do mode changes here (either also or instead of above, ie could set and clear a flag)

    Mode next_mode;

    switch (_mode) {
        case Mode::STARTUP:
            next_mode = step_startup();
            break;

        case Mode::REST:
            next_mode = step_rest();
            break;

        case Mode::TROT:
            next_mode = step_trot();
            break;

        default:
            next_mode = _mode;
            break;
    }

    _mode = next_mode;
    _ticks++;
    return _joint_angles;
}

// Private Methods

// Step startup mode forward one time step
QuadControl::Mode QuadControl::step_startup() {
    // Calculate joint offsets for this time step, +/- towards default stance + default height

    // increment _joint_angles with rotation step

    // Hip roll handling
    for (std::size_t i = 0; i < common::LEG_COUNT; i++) {
        double step;
        // Find direction towards goal
        if (_joint_angles.body_joint_angles[i].hip_roll < _startup_goal.body_joint_angles[i].hip_roll) {
            step = config::startup_joint_step;
        } else {
            step = -config::startup_joint_step;
        }

        // Prevent overshoot
        if (abs(_startup_goal.body_joint_angles[i].hip_roll - _joint_angles.body_joint_angles[i].hip_roll) <= abs(step)) {
            _joint_angles.body_joint_angles[i].hip_roll = _startup_goal.body_joint_angles[i].hip_roll;
        } else {
            _joint_angles.body_joint_angles[i].hip_roll += step;
        }
    } 
    // Hip and knee pitch handling

    return Mode::STARTUP;
}

// Step rest mode forward one time step
QuadControl::Mode QuadControl::step_rest() {
    // step logic
    for (std::size_t i = 0; i < common::LEG_COUNT; i++) {
        _foot_locations[i] = quad::config::DEFAULT_STANCE[i] + Eigen::Vector3d(0.0, 0.0, _height);
    }
    _joint_angles = body_inverse_kinematics(_foot_locations);

    // check for mode change
    return (_command.is_toggle_mode)? Mode::TROT : Mode::REST;
}

// Step gait sequence forward one time step
QuadControl::Mode QuadControl::step_trot() {
    // step logic
    for (std::size_t i = 0; i < common::LEG_COUNT; i++) {
        // Given the contact phase (swing or overlap) find the contact mode (swing or stance) for this leg
        int contact_mode = config::contact_phases[contact_phase()][i];

        if (contact_mode == config::STANCE) {
            // Apply next stance location to foot
            stance_next_foot_location(_foot_locations[i]);
        // Swing
        } else {
            // How far into the swing we are (0->1 where 0 is start and 1 is completion)
            double swing_proportion = static_cast<double>(contact_phase_depth()) / 
                                      static_cast<double>(config::swing_ticks);

            // Apply next swing location to foot
            swing_next_foot_location(_foot_locations[i], swing_proportion, i);
        }
    }
    _joint_angles = body_inverse_kinematics(_foot_locations);

    // check for mode change
    return (_command.is_toggle_mode)? Mode::REST : Mode::TROT;
}


BodyJointAngles QuadControl::body_inverse_kinematics(const std::array<Eigen::Vector3d, quad::common::LEG_COUNT>& targets) {
    BodyJointAngles angles;

    // Inverse kinematics on each leg target, subtracting by hip origin to compute ik about the legs origin
    for (std::size_t i = 0; i < targets.size(); i++) {
        angles.body_joint_angles[i] = leg_inverse_kinematics(targets[i] - config::LEG_HIP_ORIGINS[i], i);
    }

    return angles;
}


// Calculates the leg joint angles needed to reach a target position with respect to the legs hip origin.
// NOTE: Core calculation assumes a front left leg with each joints positive rotation to go counter clock wise,
// this is compensated for by a helper function to work for all legs.
LegJointAngles QuadControl::leg_inverse_kinematics(const Eigen::Vector3d& target, std::size_t leg_index) {
    constexpr double ACOS_CLAMP = 0.999999;

    // ---- Looking head on into y-z plane -------

    // Distance from leg origin to foot target position in y-z plane
    double d_origin_foot_yz = std::sqrt(sq(target.y()) + sq(target.z()));

    // Distance from hip to foot in y-z plane
    double d_hip_foot_yz = std::sqrt(sq(d_origin_foot_yz) - sq(config::ABDUCTION_OFFSET));

    // Internal angle formed by right triangle formed by d_origin_foot_yz and d_hip_foot_yz
    // and the abduction offset. (Offset may need to change sign depending on which leg)
    double cos_phi = config::ABDUCTION_OFFSETS[leg_index] / d_origin_foot_yz;
    cos_phi = std::clamp(cos_phi, -ACOS_CLAMP, ACOS_CLAMP);
    double phi = std::acos(cos_phi);

    // Angle of vector from leg origin to foot target wrt positive y-axis
    double origin_foot_angle = std::atan2(target.z(), target.y());

    // Ab/Adduction angle, relative to positive y-axis
    double hip_roll = phi + origin_foot_angle;


    // ---- Looking at side of leg normal to tilted z-axis -------
    
    // Angle between tilted negative z-axis and the hip to foot vector
    double theta = (leg_index == common::FL || leg_index == common::FR)?
                   std::atan2(-target.x(), d_hip_foot_yz) : std::atan2(target.x(), d_hip_foot_yz);

    // Distance between hip and foot
    double d_hip_foot = std::sqrt(sq(d_hip_foot_yz) + sq(target.x()));

    // Angle between hip to foot vector and link L1
    double cos_trident = (sq(config::L1) + sq(d_hip_foot) - sq(config::L2)) / 
                         (2 * config::L1 * d_hip_foot);
    cos_trident = std::clamp(cos_trident, -ACOS_CLAMP, ACOS_CLAMP);
    double trident = std::acos(cos_trident);

    // Angle of link L1 wrt the tilted negative z-axis (+/- makes hip bend backward/forward)
    // must be opposite sign of knee_pitch for valid solution 
    double hip_pitch = (leg_index == common::FL || leg_index == common::FR)? 
                       (theta + trident) : -(theta + trident);

    // Angle between links L1 and L2
    double cos_beta = (sq(config::L1) + sq(config::L2) - sq(d_hip_foot)) / 
                      (2 * config::L1 * config::L2);
    cos_beta = std::clamp(cos_beta, -ACOS_CLAMP, ACOS_CLAMP);
    double beta = std::acos(cos_beta);

    // Angle of link L2 wrt hip pitch (+/- makes knee bend forward/backward)
    // must be opposite sign of hip_pitch for valid solution 
    double knee_pitch = (leg_index == common::FL || leg_index == common::FR)?
                        -(std::numbers::pi - beta) : (std::numbers::pi - beta);

    LegJointAngles leg_joint_angles = LegJointAngles {.hip_roll   = hip_roll, 
                                                      .hip_pitch  = hip_pitch,
                                                      .knee_pitch = knee_pitch};

    correct_joint_signs(leg_joint_angles, leg_index);

    return leg_joint_angles;
}

// Corrects the sign of leg joint angles per each legs unique rotation axis.
// NOTE: It is assumed that all joints by default rotate positively in the counter
// clock wise direction
void QuadControl::correct_joint_signs(LegJointAngles& angles, std::size_t leg_index) {
    // For back legs hip roll must go opposite direction than front legs since they 
    // are mounted reversed (except for back left)
    //
    // For right side legs hip pitch and knee pitch must go opposite direction than
    // left side legs
    if (leg_index == common::FR) {
        angles.hip_roll   = -angles.hip_roll;
        angles.hip_pitch  = -angles.hip_pitch;
        angles.knee_pitch = -angles.knee_pitch;

    } else if (leg_index == common::BR) {
        angles.hip_roll   = -angles.hip_roll;
        angles.hip_pitch  = -angles.hip_pitch;
        angles.knee_pitch = -angles.knee_pitch;
    }
}


Eigen::Vector3d QuadControl::leg_forward_kinematics(const LegJointAngles& angles) {
    Eigen::Vector3d result;
    
    // Equation taken from derived forward kinematics from scripts/python/kinematics.py
    
    result.x() = -config::L1 * std::sin(angles.hip_pitch) - 
                  config::L2 * std::sin(angles.hip_pitch + angles.knee_pitch);

    result.y() = config::ABDUCTION_OFFSET * std::cos(angles.hip_roll) + 
                 config::L1 * std::sin(angles.hip_roll) * std::cos(angles.hip_pitch) + 
                 config::L2 * std::sin(angles.hip_roll) * std::cos(angles.hip_pitch + angles.knee_pitch);

    result.z() = config::ABDUCTION_OFFSET * std::sin(angles.hip_roll) -
                 config::L1 * std::cos(angles.hip_roll) * std::cos(angles.hip_pitch) -
                 config::L2 * std::cos(angles.hip_roll) * std::cos(angles.hip_pitch + angles.knee_pitch);

    return result;
}


// Apply next foot location for stance contact mode
void QuadControl::stance_next_foot_location(Eigen::Vector3d& foot_location) {
    // Calculate inverse of commanded body velocity for x-y (z is not taken as inverse)
    Eigen::Vector3d inv_vel_xy(-_command.horizontal_velocity_x, 
                               -_command.horizontal_velocity_y, 
                               (_height - foot_location.z()) / config::z_time_constant);
    // Get inverse position delta for this time step
    Eigen::Vector3d inv_pos_delta_xy = inv_vel_xy * common::DT;


    // Calculate inverse body rotation delta for this time step to achieve commanded yaw rate
    Eigen::Matrix3d inv_rot_delta_z = Eigen::AngleAxisd(-_command.yaw_rate * common::DT, 
                                                  Eigen::Vector3d::UnitZ()).toRotationMatrix();


    // Apply rotational and positional deltas to current foot positions
    foot_location = inv_rot_delta_z * foot_location + inv_pos_delta_xy;
}


// Where to move the leg in x-y for the swing contact mode
Eigen::Vector3d QuadControl::swing_raibert_touchdown_location(std::size_t leg_index) {
    // Positional deltas x-y for current time step to achieve desired body velocity from commanded horizontal velocities
    Eigen::Vector3d vel_xy(_command.horizontal_velocity_x, _command.horizontal_velocity_y, 0);

    Eigen::Vector3d pos_delta_xy = vel_xy * 
                                   config::alpha * 
                                   config::stance_ticks * common::DT;
    // Rotational delta in z for current time step to achieve desired body yaw from commanded yaw rate
    Eigen::Matrix3d rot_delta_z = Eigen::AngleAxisd(_command.yaw_rate * 
                                                    config::beta *
                                                    config::stance_ticks * common::DT, 
                                                    Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // Apply rotational and positional deltas to default stance
    return rot_delta_z * config::DEFAULT_STANCE[leg_index] + pos_delta_xy;
}


// Apply next foot location for swing contact mode
void QuadControl::swing_next_foot_location(Eigen::Vector3d& foot_location, double swing_proportion, std::size_t leg_index) {
    // Calculate swing height based on how far into the swing we are
    double swing_height = 0.0;
    if (swing_proportion < 0.5) {
        // Triangular ramp up to swing height delta
        swing_height = config::z_delta_swing_height * (swing_proportion / 0.5);
    } else {
        // Triangular ramp down from swing height delta
        swing_height = config::z_delta_swing_height * ((1 - swing_proportion ) / 0.5);
    }

    // Calculate raibert touchdown location
    Eigen::Vector3d touchdown_location = swing_raibert_touchdown_location(leg_index);

    // Velocity needed to get to touchdown location within the time left in the swing
    double time_left = common::DT * (config::swing_ticks * (1.0 - swing_proportion));
    Eigen::Vector3d vel = (touchdown_location - foot_location) / time_left;
    vel.z() = 0.0;

    // Position delta for this time step
    Eigen::Vector3d pos_delta_xy = vel * common::DT;

    // Take z position as space between robots height and the calculated swing height
    // z position is the current height plus offset either up/down by swing height
    foot_location.z() = _height + swing_height;
    // Apply positional deltas to x-y of foot location
    foot_location = foot_location + pos_delta_xy;
}


// Find what the current contact phase is (swing or overlap) given how deep we are into the total gait
int QuadControl::contact_phase() {
    // How many ticks deep within overall gait
    int gait_tick_depth = _ticks % config::total_gait_ticks;
    int phase_tick_sum = 0;
    int i = 0;

    // Find what the current contact phase is (swing or overlap) given how deep we are into the total gait
    for (i = 0; i < config::PHASE_COUNT; i++) {
        phase_tick_sum += config::phase_ticks[i];

        if (phase_tick_sum > gait_tick_depth) {
            break;
        }
    }

    // NOTE: A sign that gait tuning/timing configurations are messed up
    assert(((0 <= i) && (i < config::PHASE_COUNT)) && "contact_phase: i is not valid!");

    return i;
}


// How many ticks deep into current contact phase (swing or overlap) since its start
int QuadControl::contact_phase_depth() {
    // How many ticks deep within overall gait
    int gait_tick_depth = _ticks % config::total_gait_ticks;
    int phase_tick_sum = 0;
    double phase_tick_depth = 0.0;
    int i = 0;

    // Find what the current contact phase is (swing or overlap) given how deep we are into the total gait
    for (i = 0; i < config::PHASE_COUNT; i++) {
        phase_tick_sum += config::phase_ticks[i];

        if (phase_tick_sum > gait_tick_depth) {
            // How many ticks deep into current phase (swing or overlap) since its start 
            phase_tick_depth = gait_tick_depth - (phase_tick_sum - config::phase_ticks[i]);
            break;
        }
    }

    // NOTE: A sign that gait tuning/timing configurations are messed up
    assert(((0 <= i) && (i < config::PHASE_COUNT)) && "contact_phase_depth: i is not valid!");

    return phase_tick_depth;
}


