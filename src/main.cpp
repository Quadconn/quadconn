#include <iostream>

#include "Manipulator.hpp"

int main() {

    Manipulator manip(190, 190);

    Manipulator::JointAngles jAngles;
    switch (manip.ik(50, 25, jAngles)) {
        case Manipulator::Status::SUCCESS:
            std::cout << "SUCCESS: \n" 
                      << "theta1_p = " << jAngles.theta1_p << "\n"
                      << "theta2_p = " << jAngles.theta2_p << "\n"
                      << "theta1_n = " << jAngles.theta1_n << "\n"
                      << "theta2_n = " << jAngles.theta2_n << "\n";
            break;
        case Manipulator::Status::OUT_OF_JOINT_LIMITS:
            std::cout << "OUT_OF_JOINT_LIMITS\n";
            break;
        case Manipulator::Status::UNREACHABLE:
            std::cout << "UNREACHABLE\n";
            break;
    }

    return 0;
}
