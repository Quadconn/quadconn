#pragma once

#include <cstddef>

#include <Eigen/Dense>

#include "quad_config.hpp"
#include "joint_angles.hpp"
#include "command.hpp"

class QuadControl {
    public:
        QuadControl() : 
            _foot_locations(quad::config::DEFAULT_STANCE)
        {};

        // Set the stored command for use in gait calculations
        void set_command(const Command& command);

        // Advance the gait sequence for this time step
        BodyJointAngles step_gait();



    private:
        Command _command;
        std::array<Eigen::Vector3d, quad::common::LEG_COUNT> _foot_locations;
        int _ticks = 0;

        BodyJointAngles body_inverse_kinematics(const std::array<Eigen::Vector3d, quad::common::LEG_COUNT>& targets);

        LegJointAngles leg_inverse_kinematics(const Eigen::Vector3d& target, std::size_t leg_index);

        Eigen::Vector3d leg_forward_kinematics(const LegJointAngles& angles);

        void stance_next_foot_location(Eigen::Vector3d& foot_location);

        Eigen::Vector3d swing_raibert_touchdown_location(std::size_t leg_index);

        void swing_next_foot_location(Eigen::Vector3d& foot_location, double swing_proportion, std::size_t leg_index);

        int contact_phase();

        int contact_phase_depth();
};
