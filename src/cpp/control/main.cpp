#include <iostream>

#include "leg_ik.hpp"


constexpr double ABDUCTION_OFFSET = 0.04241 + 0.069;
constexpr double L1  = 0.19425;
constexpr double L2  = 0.140;

int main() {

    Leg leftLeg;

    if (leg_ik(leftLeg, L1 + L2, ABDUCTION_OFFSET, 0)) {
        std::cout << leftLeg.abduction_angle << std::endl
                  << leftLeg.hip_angle << std::endl
                  << leftLeg.knee_angle << std::endl;
    }
    return 0;
}
