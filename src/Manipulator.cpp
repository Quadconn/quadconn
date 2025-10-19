#include "Manipulator.hpp"

#include <cmath>

bool Manipulator::outOfJointLimits(JointAngles& j) {
    return (j.theta1_p > _l1_max_angle) || (j.theta2_p > _l2_max_angle) || 
           (j.theta1_n < _l1_min_angle) || (j.theta2_n < _l2_min_angle);
}

Manipulator::Status Manipulator::ik(double x, double y, JointAngles& jOut) {
    double c2 = (std::pow(x, 2) + std::pow(y, 2) 
                 - std::pow(_l1, 2) - std::pow(_l2, 2)) / (2 * _l1 * _l2);

    if ((c2 < -1.0) || (c2 > 1.0)) {
        return Status::UNREACHABLE;
    }

    double s2 = std::sqrt(1 - std::pow(c2, 2));
    jOut.theta2_p = std::atan2(s2, c2);
    jOut.theta2_n = std::atan2(-s2, c2);

    double k1 = _l1 + _l2 * c2;
    double k2_p = _l2 * s2;
    double k2_n = _l2 * -s2;

    jOut.theta1_p = std::atan2(y, x) - std::atan2(k2_p, k1);
    jOut.theta1_n = std::atan2(y, x) - std::atan2(k2_n, k1);

    if (outOfJointLimits(jOut)) {
        return Status::OUT_OF_JOINT_LIMITS;
    }

    return Status::SUCCESS;
}
