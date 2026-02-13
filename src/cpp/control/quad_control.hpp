#pragma once

#include <Eigen/Dense>

#include "quad_config.hpp"
#include "joint_angles.hpp"
#include "command.hpp"

class QuadControl {
    public:
        QuadControl() : 
            _front_left_foot_location(QuadConfig::default_front_left_foot_location)
        {};

        // Set the stored command for use in gait calculations
        void set_command(const Command& command);

        // Advance the gait sequence for this time step
        JointAngles step_gait();



    private:
        Command _command;
        Eigen::Vector3d _front_left_foot_location;
        int _ticks = 0;

        JointAngles leg_inverse_kinematics(const Eigen::Vector3d& target);

        Eigen::Vector3d leg_forward_kinematics(const JointAngles& angles);

        void stance_next_foot_location(Eigen::Vector3d& foot_location);

        Eigen::Vector3d swing_raibert_touchdown_location();

        void swing_next_foot_location(Eigen::Vector3d& foot_location, double swing_proportion);

        int contact_phase();

        int contact_phase_depth();
};
