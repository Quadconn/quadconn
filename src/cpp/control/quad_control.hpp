#pragma once

#include <Eigen/Dense>

#include "quad_config.hpp"
#include "joint_angles.hpp"
#include "command.hpp"

class QuadControl {
    public:
        QuadControl() : 
            _front_left_foot_location(0.0, QuadConfig::ABDUCTION_OFFSET, 
                                      -(QuadConfig::L1 + (QuadConfig::L2 / 2)))
        {};
        void set_command(const Command& command);
        Eigen::Vector3d step_gait();
        JointAngles leg_inverse_kinematics(const Eigen::Vector3d& target);
        Eigen::Vector3d leg_forward_kinematics(const JointAngles& angles);
    private:
        Command _command;
        Eigen::Vector3d _front_left_foot_location;
        double _height;
        double _ticks;

        void stance_next_foot_location(Eigen::Vector3d& foot_location);
        void swing_next_foot_location(Eigen::Vector3d& foot_location, double swing_proportion);
        Eigen::Vector3d swing_raibert_touchdown_location();
        double subphase_ticks();
};
