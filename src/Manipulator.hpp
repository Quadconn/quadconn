#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <limits>

class Manipulator {
    public:
        struct JointAngles {
            double theta1_p;
            double theta2_p;
            double theta1_n;
            double theta2_n;
        };

        enum class Status {
            UNREACHABLE,
            OUT_OF_JOINT_LIMITS,
            SUCCESS,
        };

        Manipulator(double l1, double l2, 
                    double l1_max_angle, double l1_min_angle,
                    double l2_max_angle, double l2_min_angle) 
            : _l1(l1), _l2(l2), 
              _l1_max_angle(l1_max_angle), _l1_min_angle(l1_min_angle),
              _l2_max_angle(l2_max_angle), _l2_min_angle(l2_min_angle) {};

        Manipulator(double l1, double l2)
            : _l1(l1), _l2(l2), 
              _l1_max_angle(std::numeric_limits<double>::infinity()),
              _l1_min_angle(-std::numeric_limits<double>::infinity()),
              _l2_max_angle(std::numeric_limits<double>::infinity()), 
              _l2_min_angle(-std::numeric_limits<double>::infinity()) {};

        // Return value tells if point is reachable
        Status ik(double x, double y, JointAngles& j);

    private:
        double _l1;
        double _l2;
        double _l1_max_angle;
        double _l1_min_angle;
        double _l2_max_angle;
        double _l2_min_angle;

    bool outOfJointLimits(JointAngles& j);

};

#endif
