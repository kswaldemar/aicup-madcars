//
// Created by valdemar on 31.08.18.
//

#pragma once

#include <cmath>
#include <chipmunk/cpVect.h>

static constexpr double EPS = 0.00001;
static constexpr double INF = 1e9;

constexpr inline bool eps_eq(double v1, double v2) {
    return (v1 - v2 < EPS) && (v1 - v2 > -EPS);
}

struct vec2 {
    constexpr vec2() = default;

    constexpr vec2(double x_, double y_)
        : x(x_)
        , y(y_) {
    }

    constexpr vec2(cpVect v)
        : x(v.x)
        , y(v.y) {
    }

    constexpr inline operator cpVect() const {
        return {x, y};
    }

    constexpr inline vec2 &operator+=(const vec2 &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    constexpr inline vec2 &operator-=(const vec2 &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    constexpr inline vec2 operator+(const vec2 &other) const {
        return vec2(*this) += other;
    }

    constexpr inline vec2 operator-(const vec2 &other) const {
        return vec2(*this) -= other;
    }

    constexpr inline vec2 operator-() const {
        return {-x, -y};
    }

    constexpr inline vec2 &operator*=(double val) {
        x *= val;
        y *= val;
        return *this;
    }

    constexpr inline vec2 &operator/=(double val) {
        return (*this) *= (1.0 / val);
    }

    constexpr inline vec2 operator*(double val) const {
        return vec2{x * val, y * val};
    }

    constexpr inline vec2 operator/(double val) const {
        return vec2{x / val, y / val};
    }

    constexpr inline vec2 operator*(const vec2 &other) const {
        return {x * other.x, y * other.y};
    }

    constexpr inline vec2 operator/(const vec2 &other) const {
        return {x / other.x, y / other.y};
    }

    constexpr inline bool operator==(const vec2 &other) const {
        return eps_eq(x, other.x) && eps_eq(y, other.y);
    }

    constexpr inline bool operator!=(const vec2 &other) const {
        return x != other.x || y != other.y;
    }

    constexpr inline double len2() const {
        return x * x + y * y;
    }

    double len() const {
        return std::sqrt(x * x + y * y);
    }

    double x = 0.0;
    double y = 0.0;
};

inline vec2 normalize(vec2 v) {
    double norm = v.len();
    if (!eps_eq(norm, 0.0)) {
        return v / norm;
    }
    return {};
}

inline vec2 rot90(vec2 v) {
    return {-v.y, v.x};
}

constexpr inline bool norm_leq(const vec2 &dist, double r) {
    return dist.len2() <= r * r;
}

inline double length(vec2 v) {
    return v.len();
}

constexpr inline double dot(vec2 v1, vec2 v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

constexpr inline double cross(vec2 v1, vec2 v2) {
    return v1.x * v2.y - v1.y * v2.x;
}
