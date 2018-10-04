//
// Created by valdemar on 02.09.18.
//

#include "vis_debug.h"
#include "RewindClient.h"
#include "../rotation.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

void draw_segment(const Segment &seg, uint32_t color, size_t layer) {
    const auto perp = rot90(normalize(seg.p2 - seg.p1)) * seg.height;

    VIS_LINE(seg.p1 + perp, seg.p2 + perp, color, layer);
    VIS_LINE(seg.p1 - perp, seg.p2 - perp, color, layer);
    VIS_LINE(seg.p1 + perp, seg.p1 - perp, color, layer);
    VIS_LINE(seg.p2 + perp, seg.p2 - perp, color, layer);
}

void draw_contour(const std::vector <vec2> &points, uint32_t color, size_t layer, bool closed) {
    for (size_t i = 1; i < points.size(); ++i) {
        VIS_LINE(points[i - 1], points[i], color, layer);
    }
    if (closed) {
        VIS_LINE(points.back(), points.front(), color, layer);
    }
}

void draw_box(vec2 center, double half_size, double angle, uint32_t color, size_t layer) {
    auto rot = ccw_rotation(angle);
    vec2 vx = rot(vec2(half_size, 0));
    vec2 vy = rot(vec2(0, half_size));

    VIS_LINE(center + vx + vy, center + vx - vy, color, layer);
    VIS_LINE(center + vx - vy, center - vx - vy, color, layer);
    VIS_LINE(center - vx - vy, center - vx + vy, color, layer);
    VIS_LINE(center - vx + vy, center + vx + vy, color, layer);
}

void draw_car(const ProtoCar &proto, const CarDescription &car, size_t layer) {
    auto rot = ccw_rotation(car.body.angle);

    const uint32_t car_color = 0xff8300;
    auto car_points = proto.body_poly;
    for (auto &p : car_points) {
        p.x *= car.x_modification;
        p = car.body.origin + rot(p);
    }
    draw_contour(car_points, car_color, layer);

    const uint32_t button_color = 0x00ff00;
    auto button_points = proto.button_poly;
    for (auto &p : button_points) {
        p.x *= car.x_modification;
        p = car.body.origin + rot(p);
    }
    draw_contour(button_points, button_color, layer);

    const uint32_t wheel_color = 0x4300ff;
    const uint32_t wheel_mark_color = 0x09ea81;
    if (proto.squared_wheels) {
        draw_box(car.front_wheel.origin, proto.front_wheel.radius, car.front_wheel.angle, wheel_color, layer);
        draw_box(car.rear_wheel.origin, proto.rear_wheel.radius, car.rear_wheel.angle, wheel_color, layer);
    } else {
        auto f_rot = ccw_rotation(car.front_wheel.angle);
        VIS_CIRCLE(car.front_wheel.origin, proto.front_wheel.radius, wheel_color, layer);
        VIS_LINE(car.front_wheel.origin, car.front_wheel.origin + f_rot(vec2(proto.front_wheel.radius, 0)),
                 wheel_mark_color, layer);

        auto r_rot = ccw_rotation(car.rear_wheel.angle);
        VIS_CIRCLE(car.rear_wheel.origin, proto.rear_wheel.radius, wheel_color, layer);
        VIS_LINE(car.rear_wheel.origin, car.rear_wheel.origin + r_rot(vec2(proto.rear_wheel.radius, 0)),
                 wheel_mark_color, layer);
    }
}

void draw_game(const Game &game, size_t layer) {
    const uint32_t color = 0x224242;
    for (const auto &segment : game.proto_map) {
        draw_segment(segment, color, layer);
    }
}

void draw_world(const Game &game, const World &world, size_t layer) {
    draw_car(game.proto_car, world.cars[0], layer);
    draw_car(game.proto_car, world.cars[1], layer);

    VIS_LINE(0, world.deadline_mark, 1200, world.deadline_mark, 0xff003b, layer);
}

#pragma GCC diagnostic pop