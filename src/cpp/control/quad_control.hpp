#pragma once

#include <cstddef>

#include <Eigen/Dense>

#include "quad_config.hpp"
#include "joint_angles.hpp"
#include "quad_command.hpp"

class QuadControl {
    public:
        QuadControl() : 
            _foot_locations(quad::config::DEFAULT_STANCE)
        {};

        // Set the stored command for use in gait calculations
        void set_command(const QuadCommand& command);

        // Advance the robot control for this time step
        BodyJointAngles step();


    private:

        enum class Mode{
            STARTUP,
            REST,
            TROT,
        };


        QuadCommand _command;
        std::array<Eigen::Vector3d, quad::common::LEG_COUNT> _foot_locations;
        int _ticks = 0;
        // This is the distance of the foot down from the body of the robot
        // (+) -> above body
        // (0) -> equal to body
        // (-) -> below body
        double _height = -(quad::config::L1 + (quad::config::L2 / 2));
        Mode _mode = Mode::STARTUP;


        void step_gait();

        BodyJointAngles body_inverse_kinematics(const std::array<Eigen::Vector3d, quad::common::LEG_COUNT>& targets);

        LegJointAngles leg_inverse_kinematics(const Eigen::Vector3d& target, std::size_t leg_index);

        void correct_joint_signs(LegJointAngles& angles, std::size_t leg_index);

        Eigen::Vector3d leg_forward_kinematics(const LegJointAngles& angles);

        void stance_next_foot_location(Eigen::Vector3d& foot_location);

        Eigen::Vector3d swing_raibert_touchdown_location(std::size_t leg_index);

        void swing_next_foot_location(Eigen::Vector3d& foot_location, double swing_proportion, std::size_t leg_index);

        int contact_phase();

        int contact_phase_depth();
};
