//
// Created by valdemar on 10.09.18.
//

#pragma once

#include "../structures.h"
#include "../simulation/simulator.h"

class Evaluator {
public:
    ///For initialization only
    Evaluator(const Game &game, const Simulator &sim);

    double eval(const NativeWorld &world, bool vis = false) const;

private:
    struct config_t {
        double my_inclination = 0;
        double enemy_inclination = 0;
        double btn_bonus = 0;
        double btn_penalty = 0;
        double en_impact = 0;
        double my_impact = 0;
        double my_speed_bonus = 0;
        double height_diff = 0;
        double height_desire = 0;
    };

    enum CarType {
        BUGGY = 1,
        BUS = 2,
        SQ_BUGGY = 3,
    };

    void init_config();

    CarType car_type_;

    //Local car shifts
    vec2 center_;
    vec2 back_;
    vec2 front_;
    std::vector<vec2> car_points_;
    std::vector<vec2> button_shape_pts_;

    int map_id_;

    vec2 button_dir_; //Direction with width
    vec2 button_dir_norm_;

    config_t c_;
};