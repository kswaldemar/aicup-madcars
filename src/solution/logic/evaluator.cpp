//
// Created by valdemar on 10.09.18.
//

#include "evaluator.h"
#include "../rotation.h"
#include "../math_utils.h"
#include "../common/RewindClient.h"

#include <chipmunk/chipmunk_structs.h>

namespace {

///Button dir should be norm vector
inline double rel_dist_norm(vec2 x_dir, vec2 y_dir, vec2 obj_pt) {
    double x_component = dot(obj_pt, x_dir);
    double y_component = dot(obj_pt, y_dir);
    if (y_component < 0) {
        return 0.0;
    }
    //Clamped to 500px
    const double dist = obj_pt.len();
    return std::max(1.0 - dist / 500.0, 0.0) * (1.0 - std::abs(x_component / dist));
}

} // anonymous namespace

Evaluator::Evaluator(const Game &game, const Simulator &sim) {
    if (game.proto_car.external_id == 2) {
        //Bus
        button_dir_ = ccw_rotation(atan2(-3.0, 1.7))({28, 0});
    } else {
        button_dir_ = {38, 0};
    }

    button_dir_norm_ = normalize(button_dir_);

    center_ = cpBodyGetCenterOfGravity(sim.cars_[0].body);
    back_ = {5, center_.y};
    front_ = {150, center_.y};

    const vec2 wheel_pt_lowering = {0, 5};
    const auto car_abs_pos = cpBodyGetPosition(sim.cars_[0].body);
    car_points_.emplace_back(back_);
    car_points_.emplace_back(front_);
    car_points_.emplace_back(cpBodyGetPosition(sim.cars_[0].rear_wheel.body) - wheel_pt_lowering - car_abs_pos);
    car_points_.emplace_back(cpBodyGetPosition(sim.cars_[0].front_wheel.body) - wheel_pt_lowering - car_abs_pos);

    for (int i = 0; i < cpPolyShapeGetCount(sim.cars_[0].button); ++i) {
        button_shape_pts_.emplace_back(cpPolyShapeGetVert(sim.cars_[0].button, i));
    }

    car_type_ = static_cast<CarType>(game.proto_car.external_id);
    map_id_ = game.proto_map_external_id;

    init_config();
}

double Evaluator::eval(const NativeWorld &w, bool vis) const {
    using namespace std;

    const auto my_translate = [&w](const vec2 pt) {
        return cpBodyLocalToWorld(w.me().body, pt);
    };
    const auto en_translate = [&w](const vec2 pt) {
        return cpBodyLocalToWorld(w.enemy().body, pt);
    };

    const auto rot = ccw_rotation(cpBodyGetRotation(w.me().body));
    const auto rot2 = ccw_rotation(cpBodyGetRotation(w.enemy().body));

    const cpVect my_origin = cpBodyLocalToWorld(w.me().body, {0, 0});
    const cpVect en_origin = cpBodyLocalToWorld(w.enemy().body, {0, 0});

    const double my_angle_norm = constraint_angle(cpBodyGetAngle(w.me().body));
    const double en_angle_norm = constraint_angle(cpBodyGetAngle(w.enemy().body));
    const vec2 my_button_center = cpBodyLocalToWorld(w.me().body, cpShapeGetCenterOfGravity(w.me().button));
    const vec2 en_button_center = cpBodyLocalToWorld(w.enemy().body, cpShapeGetCenterOfGravity(w.enemy().button));

    const vec2 my_center = my_translate(cpBodyGetCenterOfGravity(w.me().body));
    const vec2 en_center = en_translate(cpBodyGetCenterOfGravity(w.enemy().body));
    const vec2 my_mirror = w.my_mirror();
    const vec2 en_mirror = w.en_mirror();

    //Penalty for inclination
    double k_my_inclination = lerp_clamp(to_deg(abs(my_angle_norm)), 70, 180, 0.0, 1.0);
    double k_enemy_inclination = lerp_clamp(to_deg(abs(en_angle_norm)), 70, 180, 0.0, 1.0);

    ///Penalty when enemy near my button from side where he can touch it
    const vec2 my_btn_vec = rot(button_dir_norm_ * my_mirror * vec2{my_mirror.x, my_mirror.x});
    const vec2 my_btn_perp = rot90(my_btn_vec);
    double k_button_penalty = 0.0;
    for (const auto p : car_points_) {
        k_button_penalty = std::max(k_button_penalty,
                                    rel_dist_norm(my_btn_vec, my_btn_perp,
                                                  en_translate(p * en_mirror) - my_button_center));
    }

    ///Bonus when I am near enemy button from side where I can touch it
    const vec2 enemy_btn_vec = rot2(button_dir_norm_ * en_mirror * vec2{en_mirror.x, en_mirror.x});
    const vec2 enemy_btn_perp = rot90(enemy_btn_vec);
    double k_button_bonus = 0.0;
    for (const auto p : car_points_) {
        k_button_bonus = std::max(k_button_bonus,
                                  rel_dist_norm(enemy_btn_vec, enemy_btn_perp,
                                                my_translate(p * my_mirror) - en_button_center));
    }

    ///Height difference
    double my_min_y = INF;
    double en_min_y = INF;
    for (auto p : button_shape_pts_) {
        my_min_y = min(my_min_y, my_translate(p * my_mirror).y);
        en_min_y = min(en_min_y, en_translate(p * en_mirror).y);
        if (vis) {
            VIS_CIRCLE(my_translate(p * my_mirror), 1, 0xd820b0);
            VIS_CIRCLE(en_translate(p * en_mirror), 1, 0xd820b0);
        }
    }
    const double k_height_diff = lerp_clamp(my_min_y - en_min_y, -100.0, 500.0, 0.0, 1.0);

    //Impact
    const double k_dist = std::max(1.0 - (my_center - en_center).len2() / 449'000.0, 0.0);

    const auto calc_impact = [vis](vec2 tg_point, vec2 att_point, vec2 body_dir, vec2 tg_perp) {
        if (vis) {
            VIS_LINE(tg_point, att_point, 0xff0000);
        }
        const auto attack_dir = normalize(tg_point - att_point);
        double imp = max(dot(attack_dir, body_dir), 0.0);
        imp = max(imp * dot(attack_dir, tg_perp), 0.0);
        return imp;
    };

    const double shift = 20;
    const double k_my_impact = calc_impact(
        en_center + rot2({en_mirror.x * shift * (1.0 - k_enemy_inclination) - 10, 0}),
        my_origin + rot(front_ * my_mirror),
        rot({my_mirror.x, 0}),
        enemy_btn_perp
    );

    const double k_enemy_impact = calc_impact(
        my_center + rot({my_mirror.x * shift * (1.0 - k_my_inclination) - 10, 0}),
        en_origin + rot2(front_ * en_mirror),
        rot2({en_mirror.x, 0}),
        my_btn_perp
    );

    if (vis) {
        VIS_CIRCLE(en_button_center, 3, 0xff9900);
        VIS_CIRCLE(my_button_center, 3, 0x99ff00);
        for (const auto p : car_points_) {
            VIS_CIRCLE(my_translate(p * my_mirror), 2, 0xffff00);
            VIS_CIRCLE(en_translate(p * en_mirror), 2, 0xff0000);
        }
    }

    vec2 my_speed = cpBodyGetVelocity(w.me().body);
    vec2 en_speed = cpBodyGetVelocity(w.enemy().body);

    const double my_speed_val = my_speed.len();
    const double k_my_speed = lerp_clamp(my_speed_val, 0, 500, 0.0, 1.0);

    const double my_w = cpBodyGetAngularVelocity(w.me().body);
    const double en_w = cpBodyGetAngularVelocity(w.enemy().body);

    const vec2 approach_x_vec = {my_center.x < en_center.x ? 1.0 : -1.0, 0.0};
    double k_approach_bonus = lerp(dot(my_speed / my_speed_val, approach_x_vec), -1.0, 1.0, 0.5, 1.0);

    if (vis) {
        VIS_MESSAGE("\\nma: %.3lf, ea: %.3lf\\n", to_deg(my_angle_norm), to_deg(en_angle_norm));
        VIS_MESSAGE("mw: %.3lf, ew: %.3lf\\n",
                    to_deg(abs(cpBodyGetAngularVelocity(w.me().body))),
                    to_deg(abs(cpBodyGetAngularVelocity(w.enemy().body))));
        VIS_MESSAGE("mv: %.4lf, %.4lf\\n", my_speed.x, my_speed.y);
        VIS_MESSAGE("ev: %.4lf, %.4lf\\n\\n", en_speed.x, en_speed.y);
    }

    ///Score calculation
    ///All scored values should be normalized in [0.0, 1.0]
    double score = 0.0;
    const double k_my_angular_velocity = lerp_clamp(abs(my_w), 2 * M_PI, M_PI_4, 0.0, 1.0);
    const double k_enemy_angular_velocity = lerp_clamp(abs(en_w), 2 * M_PI, M_PI_4, 0.0, 1.0);
    const double k_turn = lerp_clamp(w.turn_idx, 500, 700, 0.0, 1.0);

    score += c_.height_desire * k_height_diff * k_turn;

    score -= c_.my_inclination * k_my_inclination * k_dist * k_my_angular_velocity;
    score += c_.enemy_inclination * k_enemy_inclination * k_dist * k_enemy_angular_velocity;

    score += c_.btn_bonus * k_button_bonus;
    score -= c_.btn_penalty * k_button_penalty;

    score += c_.my_impact * k_my_impact;
    score -= c_.en_impact * k_enemy_impact;

    score += c_.my_speed_bonus * k_my_speed * k_approach_bonus;
    score += c_.height_diff * k_height_diff;

    score -= 100000.0 * w.me().loosed;
    score += 100000.0 * w.enemy().loosed;

    if (vis) {
        VIS_MESSAGE("hdes: %.6lf\\n", c_.height_desire * k_height_diff * k_turn);
        VIS_MESSAGE("min:  %.6lf\\n", c_.my_inclination * k_my_inclination * k_dist * k_my_angular_velocity);
        VIS_MESSAGE("ein:  %.6lf\\n", c_.enemy_inclination * k_enemy_inclination * k_dist * k_enemy_angular_velocity);
        VIS_MESSAGE("bb:   %.6lf\\n", c_.btn_bonus * k_button_bonus);
        VIS_MESSAGE("bp:   %.6lf\\n", c_.btn_penalty * k_button_penalty);
        VIS_MESSAGE("mi:   %.6lf\\n", c_.my_impact * k_my_impact);
        VIS_MESSAGE("ei:   %.6lf\\n", c_.en_impact * k_enemy_impact);
        VIS_MESSAGE("sb:   %.6lf\\n", c_.my_speed_bonus * k_my_speed * k_approach_bonus);
        VIS_MESSAGE("hd:   %.6lf\\n", c_.height_diff * k_height_diff);
    }

    return score;
}

void Evaluator::init_config() {
    c_.height_desire = 0;
    c_.my_inclination = 15;
    c_.enemy_inclination = 15;
    c_.en_impact = 25;
    c_.my_impact = 25;
    c_.my_speed_bonus = 30;
    c_.btn_bonus = 45;
    c_.btn_penalty = 45;
}
