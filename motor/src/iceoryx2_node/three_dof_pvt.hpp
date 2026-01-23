#include <cstdint>
#include <iostream>

struct three_dof_pvt {
    double  pos;
    double  vel;
    double  torque;
    static constexpr const char* IOX2_TYPE_NAME = "three_dof_pvt";
};

inline auto operator<<(std::ostream& stream, const three_dof_pvt& value) -> std::ostream& {
    stream << "three_dof_theta { pos: " << value.pos << ", vel: " << value.vel << ", torque: " << value.torque << " }";
    return stream;
}