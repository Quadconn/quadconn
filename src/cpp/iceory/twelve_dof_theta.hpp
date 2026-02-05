#include <cstdint>
#include <iostream>

struct twelve_dof_theta {
    double  node1_angle;
    double  node2_angle;
    double  node3_angle;
    double  node4_angle;
    double  node5_angle;
    double  node6_angle;
    double  node7_angle;
    double  node8_angle;
    double  node9_angle;
    double  node10_angle;
    double  node11_angle;
    double  node12_angle;
    static constexpr const char* IOX2_TYPE_NAME = "twelve_dof_theta";
};

inline auto operator<<(std::ostream& stream, const twelve_dof_theta& value) -> std::ostream& {
    stream << "twelve_dof_theta { "
           << "node1: " << value.node1_angle
           << ", node2: " << value.node2_angle
           << ", node3: " << value.node3_angle
           << ", node4: " << value.node4_angle
           << ", node5: " << value.node5_angle
           << ", node6: " << value.node6_angle
           << ", node7: " << value.node7_angle
           << ", node8: " << value.node8_angle
           << ", node9: " << value.node9_angle
           << ", node10: " << value.node10_angle
           << ", node11: " << value.node11_angle
           << ", node12: " << value.node12_angle
           << " }";
    return stream;
}