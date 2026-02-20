#include <cstdint>
#include <iostream>


struct GamepadData {
    int dpad_x; int dpad_y;
    int A; int B; int X; int Y;
    int Home; int Start; int Back; 
    int L3; int R3;
    double lx; double ly; double rx; double ry;
    int RB; double RT; int LB; double LT;
    static constexpr const char* IOX2_TYPE_NAME = "GamepadData";
};

inline auto operator<<(std::ostream& stream, const GamepadData& value) -> std::ostream& {
    stream << "GamepadData { dpad_x: " << value.dpad_x << ", dpad_y: " << value.dpad_y <<
            ", A: " << value.A << ", B: " << value.B << ", X: " << value.X << ", Y: " << value.Y <<
            ", Home: " << value.Home << ", Start: " << value.Start << ", Back: " << value.Back <<
            ", L3: " << value.L3 << ", R3: " << value.R3 <<
            ", lx: " << value.lx << ", ly: " << value.ly << ", rx: " << value.rx << ", ry: " << value.ry <<
            ", RB: " << value.RB << ", RT: " << value.RT << ", LB: " << value.LB << ", LT: " << value.LT << " }";
    return stream;
}