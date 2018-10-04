//
// Created by valdemar on 09.09.18.
//

#pragma once

#include <cmath>
#include <algorithm>

///Convert angle to [-pi, pi]
inline double constraint_angle(double angle_rad) {
    double x = fmod(angle_rad + M_PI, 2.0 * M_PI);
    if (x < 0) {
        x += 2.0 * M_PI;
    }
    return x - M_PI;
}

constexpr inline double to_deg(double angle_rad) {
    return angle_rad * 180.0 / M_PI;
}

constexpr inline double to_rad(double angle_deg) {
    return angle_deg * M_PI / 180.0;
}

constexpr inline double lerp(double v, double v0, double v1, double t0, double t1) {
    return (v - v0) / (v1 - v0) * (t1 - t0) + t0;
}

constexpr inline double lerp_clamp(double v, double v0, double v1, double t0, double t1) {
    double t_min = std::min(t0, t1);
    double t_max = std::max(t0, t1);
    return std::clamp(lerp(v, v0, v1, t0, t1), t_min, t_max);
}

///Value default range [0, 1]
constexpr inline double lerp(double v, double t0, double t1) {
    return lerp(v, 0, 1, t0, t1);
}

///Value default range [0, 1]
constexpr inline double lerp_clamp(double v, double t0, double t1) {
    return lerp_clamp(v, 0, 1, t0, t1);
}