#pragma once

#include "../structures.h"

#include <vector>

void draw_segment(const Segment &seg, uint32_t color, size_t layer = 3);

void draw_contour(const std::vector<vec2> &points, uint32_t color, size_t layer = 3, bool closed = true);

void draw_box(vec2 center, double half_size, double angle, uint32_t color, size_t layer = 3);

void draw_car(const ProtoCar &proto, const CarDescription &car, size_t layer = 3);

void draw_game(const Game &game, size_t layer = 3);

void draw_world(const Game &game, const World &world, size_t layer = 3);
