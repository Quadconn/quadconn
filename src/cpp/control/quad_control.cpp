#include "quad_control.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "quad_config.hpp"
#include "joint_angles.hpp"
#include "command.hpp"

// Helper

inline double sq(double a) {
    return a*a;
}

// Public Methods

void QuadControl::set_command(const Command& command) {
    _command = command;
}

Eigen::Vector3d QuadControl::step_gait() {
}


JointAngles QuadControl::leg_inverse_kinematics(const Eigen::Vector3d& target) {
    constexpr double ACOS_CLAMP = 0.999999;

    // ---- Looking head on into y-z plane -------

    // Distance from leg origin to foot target position in y-z plane
    double d_origin_foot_yz = std::sqrt(sq(target.y()) + sq(target.z()));

    // Distance from hip to foot in y-z plane
    double d_hip_foot_yz = std::sqrt(sq(d_origin_foot_yz) - sq(QuadConfig::ABDUCTION_OFFSET));

    // Internal angle formed by right triangle formed by d_origin_foot_yz and d_hip_foot_yz
    // and the abduction offset. (Offset may need to change sign depending on which leg)
    double cos_phi = QuadConfig::ABDUCTION_OFFSET / d_origin_foot_yz;
    cos_phi = std::clamp(cos_phi, -ACOS_CLAMP, ACOS_CLAMP);
    double phi = std::acos(cos_phi);

    // Angle of vector from leg origin to foot target wrt positive y-axis
    double origin_foot_angle = std::atan2(target.z(), target.y());

    // Ab/Adduction angle, relative to positive y-axis
    double hip_roll = phi + origin_foot_angle;


    // ---- Looking at side of leg normal to tilted z-axis -------
    
    // Angle between tilted negative z-axis and the hip to foot vector
    double theta = std::atan2(-target.x(), d_hip_foot_yz);

    // Distance between hip and foot
    double d_hip_foot = std::sqrt(sq(d_hip_foot_yz) + sq(target.x()));

    // Angle between hip to foot vector and link L1
    double cos_trident = (sq(QuadConfig::L1) + sq(d_hip_foot) - sq(QuadConfig::L2)) / 
                         (2 * QuadConfig::L1 * d_hip_foot);
    cos_trident = std::clamp(cos_trident, -ACOS_CLAMP, ACOS_CLAMP);
    double trident = std::acos(cos_trident);

    // Angle of link L1 wrt the tilted negative z-axis
    double hip_pitch = theta + trident;

    // Angle between links L1 and L2
    double cos_beta = (sq(QuadConfig::L1) + sq(QuadConfig::L2) - sq(d_hip_foot)) / 
                      (2 * QuadConfig::L1 * QuadConfig::L2);
    cos_beta = std::clamp(cos_beta, -ACOS_CLAMP, ACOS_CLAMP);
    double beta = std::acos(cos_beta);

    // Angle of link L2 wrt hip pitch (+/- makes knee bend forward/backward)
    double knee_pitch = -(std::numbers::pi - beta);

    return JointAngles {.hip_roll   = hip_roll, 
                        .hip_pitch  = hip_pitch,
                        .knee_pitch = knee_pitch};
}


Eigen::Vector3d QuadControl::leg_forward_kinematics(const JointAngles& angles) {
    Eigen::Vector3d result;
    
    result.x() = -QuadConfig::L1 * std::sin(angles.hip_pitch) - 
                  QuadConfig::L2 * std::sin(angles.hip_pitch + angles.knee_pitch);

    result.y() = QuadConfig::ABDUCTION_OFFSET * std::cos(angles.hip_roll) + 
                 QuadConfig::L1 * std::sin(angles.hip_roll) * std::cos(angles.hip_pitch) + 
                 QuadConfig::L2 * std::sin(angles.hip_roll) * std::cos(angles.hip_pitch + angles.knee_pitch);

    result.z() = QuadConfig::ABDUCTION_OFFSET * std::sin(angles.hip_roll) -
                 QuadConfig::L1 * std::cos(angles.hip_roll) * std::cos(angles.hip_pitch) -
                 QuadConfig::L2 * std::cos(angles.hip_roll) * std::cos(angles.hip_pitch + angles.knee_pitch);

    return result;
}

// Private Methods

void QuadControl::stance_next_foot_location(Eigen::Vector3d& foot_location) {
    // Calculate inverse of commanded body velocity for x-y (z is not taken as inverse)
    Eigen::Vector3d inv_vel_xy(-_command.horizontal_velocity_x, 
                               -_command.horizontal_velocity_y, 
                               (_height - foot_location.z()) / QuadConfig::z_time_constant);

    // Get inverse position delta for this time step
    Eigen::Vector3d inv_pos_delta_xy = inv_vel_xy * QuadConfig::dt;


    // Calculate inverse body rotation delta for this time step to achieve commanded yaw rate
    Eigen::Matrix3d inv_rot_delta_z = Eigen::AngleAxisd(-_command.yaw_rate * QuadConfig::dt, 
                                                  Eigen::Vector3d::UnitZ()).toRotationMatrix();


    // Apply rotational and positional deltas to current foot positions
    foot_location = inv_rot_delta_z * foot_location + inv_pos_delta_xy;
}


void QuadControl::swing_next_foot_location(Eigen::Vector3d& foot_location, double swing_proportion) {
    // Calculate swing height based on how far into the swing we are
    double swing_height = 0.0;
    if (swing_proportion < 0.5) {
        // Triangular ramp up to z clearance
        swing_height = QuadConfig::z_clearance * (swing_proportion / 0.5);
    } else {
        // Triangular ramp down from z clearance
        swing_height = QuadConfig::z_clearance * ((1 - swing_proportion ) / 0.5);
    }

    // Calculate raibert touchdown location
    Eigen::Vector3d touchdown_location = swing_raibert_touchdown_location();

    // Velocity needed to get to touchdown location within the time left in the swing
    double time_left = QuadConfig::dt * (QuadConfig::swing_ticks * (1.0 - swing_proportion));
    Eigen::Vector3d vel = (touchdown_location - foot_location) / time_left;
    vel.z() = 0.0;

    // Position delta for this time step
    Eigen::Vector3d pos_delta_xy = vel * QuadConfig::dt;

    // Take z position as space between robots height and the calculated swing height
    foot_location.z() = swing_height + _command.height;
    // Apply positional deltas to x-y of foot location
    foot_location = foot_location + pos_delta_xy;
}


Eigen::Vector3d QuadControl::swing_raibert_touchdown_location() {
    // Positional deltas x-y for current time step to achieve desired body velocity from commanded horizontal velocities
    Eigen::Vector3d vel_xy(_command.horizontal_velocity_x, _command.horizontal_velocity_y, 0);

    Eigen::Vector3d pos_delta_xy = vel_xy * 
                                   QuadConfig::alpha * 
                                   QuadConfig::stance_ticks * QuadConfig::dt;
    // Rotational delta in z for current time step to achieve desired body yaw from commanded yaw rate
    Eigen::Matrix3d rot_delta_z = Eigen::AngleAxisd(_command.yaw_rate * 
                                                    QuadConfig::beta *
                                                    QuadConfig::stance_ticks * QuadConfig::dt, 
                                                    Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // Apply rotational and positional deltas to default stance
    return rot_delta_z * QuadConfig::default_front_left_foot_location + pos_delta_xy;
}

