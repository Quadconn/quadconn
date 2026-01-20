#include <cstdint>
#include <iostream>

struct target_coords {
    double x;
    double y;
    double z;
    static constexpr const char* IOX2_TYPE_NAME = "target_coords";
};

inline auto operator<<(std::ostream& stream, const target_coords& value) -> std::ostream& {
    stream << "target_coords { x: " << value.x << ", y: " << value.y << ", z: " << value.z << " }";
    return stream;
}