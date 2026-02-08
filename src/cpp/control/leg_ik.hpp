#pragma once


struct Leg {
    double hip_roll;
    double hip_pitch;
    double knee_pitch;
};

struct Point {
    double x;
    double y;
    double z;
};


inline double sq(double a) {
    return a*a;
}

bool leg_ik(Leg& out, double x, double y, double z);

void leg_fk(Point& out, double hip_roll, double hip_pitch, double knee_pitch);
