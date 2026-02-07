#include "leg_ik.hpp"

#include <cmath>
#include <algorithm>
#include <numbers>

constexpr double ABDUCTION_OFFSET = 0.04241 + 0.069;
constexpr double L1  = 0.19425;
constexpr double L2  = 0.140;

inline double sq(double a) {
    return a*a;
}

// NOTE: Currently this assumes a left sided leg
// TODO: Add any reachability checks possible
bool leg_ik(Leg& out, double x, double y, double z) {

    // ---- Looking head on into y-z plane -------

    // Distance from leg origin to foot target position in y-z plane
    double d_origin_foot_yz = std::sqrt(sq(y) + sq(z));

    // Distance from hip to foot in y-z plane
    double d_hip_foot_yz = std::sqrt(sq(d_origin_foot_yz) - sq(ABDUCTION_OFFSET));

    // Internal angle formed by right triangle formed by d_origin_foot_yz and d_hip_foot_yz
    // and the abduction offset
    double cos_phi = ABDUCTION_OFFSET / d_origin_foot_yz;
    cos_phi = std::clamp(cos_phi, -0.99, 0.99);
    double phi = std::acos(cos_phi);

    // Angle of vector from leg origin to foot target wrt positive y-axis
    double origin_foot_angle = std::atan2(z, y);

    // Ab/Adduction angle, relative to positive y-axis
    double abduction_angle = phi + origin_foot_angle;


    // ---- Looking at side of leg normal to tilted z-axis -------
    
    // Angle between tilted negative z-axis and the hip to foot vector
    double theta = std::atan2(-x, d_hip_foot_yz);

    // Distance between hip and foot
    double d_hip_foot = std::sqrt(sq(d_hip_foot_yz) + sq(x));

    // Angle between hip to foot vector and link L1
    double cos_trident = (sq(L1) + sq(d_hip_foot) - sq(L2)) / (2 * L1 * d_hip_foot);
    cos_trident = std::clamp(cos_trident, -0.99, 0.99);
    double trident = std::acos(cos_trident);

    // Angle of link L1 wrt the tilted negative z-axis
    double hip_angle = theta + trident;

    // Angle between links L1 and L2
    double cos_beta = (sq(L1) + sq(L2) - sq(d_hip_foot)) / (2 * L1 * L2);
    cos_beta = std::clamp(cos_beta, -0.99, 0.99);
    double beta = std::acos(cos_beta);

    // Angle of link L2 wrt the tilted negative z-axis
    double knee_angle = hip_angle - (std::numbers::pi - beta);

    out.abduction_angle = abduction_angle;
    out.hip_angle       = hip_angle;
    out.knee_angle      = knee_angle;

    return true;
}
