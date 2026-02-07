#pragma once


struct Leg {
    double abduction_angle;
    double hip_angle;
    double knee_angle;
};


bool leg_ik(Leg& out, double x, double y, double z);
