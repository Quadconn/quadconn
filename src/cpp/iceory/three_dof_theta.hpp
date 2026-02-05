#include <cstdint>
#include <iostream>


struct ThreeDoFTheta {
    double  theta1;
    double  theta2;
    double  theta3;
    static constexpr const char* IOX2_TYPE_NAME = "ThreeDoFTheta";
};

inline auto operator<<(std::ostream& stream, const ThreeDoFTheta& value) -> std::ostream& {
    stream << "ThreeDoFTheta { theta1: " << value.theta1 << ", theta2: " << value.theta2 << ", theta3: " << value.theta3 << " }";
    return stream;
}