#include "FastMath.hpp"

#include <cstddef>
#include <array>
#include <cmath>
#include <numbers>


namespace {

    constexpr std::array<double, 11> sinX = {
        0.0,
        0.15707963267948966,
        0.3141592653589793,
        0.47123889803846897,
        0.6283185307179586,
        0.7853981633974483,
        0.9424777960769379,
        1.0995574287564276,
        1.2566370614359172,
        1.413716694115407,
        1.5707963267948966
    };

    constexpr std::array<double, 11> sinY = {
        0.0,
        0.15643446504023087,
        0.3090169943749474,
        0.45399049973954675,
        0.5877852522924731,
        0.7071067811865475,
        0.8090169943749475,
        0.8910065241883678,
        0.9510565162951535,
        0.9876883405951378,
        1.0
    };


    template <typename T>
    T linearInterpolation(T x, T x0, T x1, T y0, T y1) {
        if (x <= x0)
            return y0;

        if (x >= x1)
            return y1;

        return y0 + ((x - x0) / (x1 - x0)) * (y1 - y0);
    }


    template <typename T, std::size_t N>
    T interpolateLUT(T x, const std::array<T, N>& lutX, const std::array<T, N>& lutY) {
        // x is under lower bound -> saturate to lowest y value
        if (x <= lutX[0]) {
            return lutY[0];
        // x is over upper bound -> saturate to highest y value
        } else if (x >= lutX[N - 1]) {
            return lutY[N - 1];
        }

        // Search for x0 <= x <= x1
        for (size_t point = 0; point < (N - 1); point++) {
            if ((lutX[point] <= x) && (lutX[point + 1] >= x)) {
                T x = x;
                T x0 = lutX[point];
                T x1 = lutX[point + 1];
                T y0 = lutY[point];
                T y1 = lutY[point + 1];
                return linearInterpolation(x, x0, x1, y0, y1);
            }
        }

        // If all checks fail return min
        return lutY[0];
    }


    double normalize_radians( double rad ) {
        rad = fmodf(rad, 2 * std::numbers::pi);

        if (rad < 0)
            return rad + (2 * std::numbers::pi);

        return rad;
    }

}

namespace fmath {

    double sin(double rad)
    {
        rad = normalize_radians(rad);

        double y = 0.0;
        
        if (rad < (std::numbers::pi / 2.0)) {
            y = interpolateLUT(rad, sinX, sinY);
        } else if (rad < std::numbers::pi) {
            y = interpolateLUT(std::numbers::pi - rad, sinX, sinY);
        } else if (rad < (std::numbers::pi + (std::numbers::pi / 2.0))) {
            y = -interpolateLUT(rad - std::numbers::pi, sinX, sinY);
        } else {
            y = -interpolateLUT((2 * std::numbers::pi) - rad, sinX, sinY);
        }

        return y;
    }


    double cos(double rad) {
        return sin((std::numbers::pi / 2.0) - rad);
    }

}
