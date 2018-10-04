//
// Created by valdemar on 01.09.18.
//

#pragma once

#include "vec2.h"

#include <cmath>

class ccw_rotation {
public:
    ccw_rotation(double angle_rad) {
        sin_a = sin(angle_rad);
        cos_a = cos(angle_rad);
    }

    ccw_rotation(vec2 v) {
        sin_a = v.y;
        cos_a = v.x;
    }

    vec2 operator()(vec2 v) const {
        return {cos_a * v.x - sin_a * v.y, sin_a * v.x + cos_a * v.y};
    }

private:
    double sin_a;
    double cos_a;
};