#include <cstdint>
#include <iostream>

struct three_dof_theta {
    double  theta1;
    double  theta2;
    double  theta3;
    static constexpr const char* IOX2_TYPE_NAME = "three_dof_theta";
};

inline auto operator<<(std::ostream& stream, const three_dof_theta& value) -> std::ostream& {
    stream << "three_dof_theta { theta1: " << value.theta1 << ", theta2: " << value.theta2 << ", theta3: " << value.theta3 << " }";
    return stream;
}