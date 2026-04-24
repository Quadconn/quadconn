#pragma once

#include <cstddef>

#include <Eigen/Dense>

#include "quad_config.hpp"
#include "quad_common.hpp"
#include "joint_angles.hpp"
#include "quad_command.hpp"

class QuadControl {
    public:
        QuadControl() 
            : _ticks(0),
              _height(-(quad::config::L1 + (quad::config::L2 / 2))),
              _mode(Mode::STARTUP),
              _joint_angles(quad::config::STARTUP_ANGLES) {

            for (std::size_t i = 0; i < quad::common::LEG_COUNT; i++) {
                _foot_locations[i] = quad::config::DEFAULT_STANCE[i] + Eigen::Vector3d(0.0, 0.0, _height);
            }

            _startup_goal = body_inverse_kinematics(_foot_locations);
        };

        // Set the stored command for use in gait calculations
        void set_command(const QuadCommand& command);

        // Advance the robot control for this time step
        BodyJointAngles step();


    private:

        enum class Mode{
            STARTUP,
            REST,
            TROT,
            SHUTDOWN,
        };


        QuadCommand _command;
        std::array<Eigen::Vector3d, quad::common::LEG_COUNT> _foot_locations;
        BodyJointAngles _joint_angles;
        int _ticks;
        // This is the distance of the foot down from the body of the robot
        // (+) -> above body
        // (0) -> equal to body
        // (-) -> below body
        double _height;
        Mode _mode;
        BodyJointAngles _startup_goal;

        Mode step_startup();

        Mode step_rest();

        Mode step_trot();

        Mode step_shutdown();

        bool hip_rolls_equal(const BodyJointAngles& a, const BodyJointAngles& b);

        bool hip_knee_pitches_equal(const BodyJointAngles& a, const BodyJointAngles& b);

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
