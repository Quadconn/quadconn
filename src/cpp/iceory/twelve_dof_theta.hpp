#include <cstdint>
#include <iostream>

struct twelve_dof_theta {
    double  theta1;
    double  theta2;
    double  theta3;
    double  theta4;
    double  theta5;
    double  theta6;
    double  theta7;
    double  theta8;
    double  theta9;
    double  theta10;
    double  theta11;
    double  theta12;
    static constexpr const char* IOX2_TYPE_NAME = "twelve_dof_theta";
};

inline auto operator<<(std::ostream& stream, const twelve_dof_theta& value) -> std::ostream& {
    stream << "twelve_dof_theta { "
           << "node1: " << value.theta1
           << ", node2: " << value.theta2
           << ", node3: " << value.theta3
           << ", node4: " << value.theta4
           << ", node5: " << value.theta5
           << ", node6: " << value.theta6
           << ", node7: " << value.theta7
           << ", node8: " << value.theta8
           << ", node9: " << value.theta9
           << ", node10: " << value.theta10
           << ", node11: " << value.theta11
           << ", node12: " << value.theta12
           << " }";
    return stream;
}