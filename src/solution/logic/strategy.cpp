//
// Created by valdemar on 01.09.18.
//

#include "strategy.h"
#include "fastrand.h"
#include "montecarlo.h"
#include "evaluator.h"

#include "../simulation/simulator.h"
#include "../common/logger.h"
#include "../common/RewindClient.h"
#include "../common/vis_debug.h"
#include "../vec2.h"
#include "../rotation.h"
#include "../math_utils.h"

#include <chipmunk/chipmunk.h>
#include <chipmunk/chipmunk_structs.h>

#include <chrono>

Strategy::~Strategy() = default;

void Strategy::next_match(Game game) {
    game_ = std::move(game);
    sim_.init(&game_);
    turn_idx_ = -1;

    evaluator_ = nullptr;
    solution_ = nullptr;
}

//MARK: move

Action Strategy::move(World world) {
    on_tick_start(world);
    sim_precision_checker(world);

    auto heurisitcs_action = heuristics_control();
    if (heurisitcs_action != Action::UNKNOWN) {
        VIS_MESSAGE("HEURISTICS %s; air %d\\n",
                    heurisitcs_action == Action::STOP ? "STOP" : heurisitcs_action == Action::LEFT ? "LEFT" : "RIGHT",
                    sim_.world_native().me().in_air);
        return on_tick_end(heurisitcs_action, world);
    }

    using montecarlo::Genome;
    if (!solution_) {
        solution_ = std::make_unique<Genome>(evaluator_.get(), true);
        enemy_solution_ = std::make_unique<Genome>(evaluator_.get(), true);
    }

    if (turn_idx_ > 0) {
        solution_->shift();
        enemy_solution_->shift();
    }

    if (turn_idx_ % Genome::TURN_LEN == 0) {
        Genome child(evaluator_.get(), false);
        sim_.save(active_buf_);

        sim_.swap_sides();
        for (int cnt = 0; cnt < 20; ++cnt) {
            enemy_solution_->mutate(&child);
            if (child.get_score(sim_, active_buf_, *solution_.get())
                > enemy_solution_->get_score(sim_, active_buf_, *solution_.get())) {
                child.duplicate(*enemy_solution_);
            }
        }
        sim_.swap_sides(); //Return me back

        for (int cnt = 0; cnt < 50; ++cnt) {
            solution_->mutate(&child);
            if (cnt < 10) {
                child.mutate();
                child.mutate();
            } else if (cnt < 30) {
                child.mutate();
            }

            if (child.get_score(sim_, active_buf_, *enemy_solution_.get())
                > solution_->get_score(sim_, active_buf_, *enemy_solution_.get())) {
                child.duplicate(*solution_);
            }
        }
    }

    return on_tick_end(solution_->get_action(), world);
}

Action Strategy::heuristics_control() {
    const auto &world = sim_.world_native();
    static constexpr int TICK_GO[] = {-1, 0, 150, 150, 180, 0, 0};
    Action ret = Action::UNKNOWN;
    if (game_.proto_car.external_id == 2
        && turn_idx_ < TICK_GO[game_.proto_map_external_id]) {
        //Force bus to go left

        if (turn_idx_ < 20 || (game_.proto_map_external_id == 6 && turn_idx_ < 120)) {
            ret = Action::STOP;
        } else if (game_.proto_map_external_id == 4) {
            if (turn_idx_ < 130) {
                ret = Action::LEFT;
            } else {
                ret = Action::STOP;
            }
        } else {
            ret = Action::LEFT;
        }
    }

    if (game_.proto_map_external_id == 5 && game_.proto_car.external_id == 1) {
        if (turn_idx_ < 30) {
            ret = Action::STOP;
        }
    }

    const bool inverted = world.my_mirror().x == -1;

    double still_angle = 67 * world.my_mirror().x;
    double dw_1 = 5;
    double dw_2 = 5;

    if (ret == Action::UNKNOWN && game_.proto_car.external_id == 2 && game_.proto_map_external_id == 1) {
        if (turn_idx_ < 45) {
            ret = Action::STOP;
        } else if (turn_idx_ < 100) {
            ret = Action::LEFT;
        } else if (turn_idx_ < 130) {
            ret = Action::STOP;
        } else if (turn_idx_ < 170) {
            ret = Action::LEFT;
        } else {
            still_angle = 79 * world.my_mirror().x;
        }
    }

    if (ret == Action::UNKNOWN && game_.proto_car.external_id == 2) {
        if (turn_idx_ < 35) {
            ret = Action::RIGHT;
        } else if (world.me().in_air) {
            const vec2 my_center = cpBodyLocalToWorld(world.me().body, cpBodyGetCenterOfGravity(world.me().body));
            const vec2 en_center = cpBodyLocalToWorld(world.enemy().body, cpBodyGetCenterOfGravity(world.enemy().body));
            const double angular_speed = cpBodyGetAngularVelocity(world.me().body);
            const double cur_angle = to_deg(constraint_angle(cpBodyGetAngle(world.me().body)));
            const double w = to_deg(angular_speed) * 0.016;

            VIS_MESSAGE("w = %.4lf\\n", w);

            bool h_control = game_.proto_map_external_id == 6;
            if (game_.proto_map_external_id == 1) {
                h_control = (inverted && en_center.x < 900) || (!inverted && en_center.x > 300);
            }
            if (!h_control) {
                double dist = (my_center - en_center).len();
                double en_vel = vec2{cpBodyGetVelocity(world.enemy().body)}.len();
                double my_vel = vec2{cpBodyGetVelocity(world.me().body)}.len();
                h_control = (my_center.y + 3 >= en_center.y) && my_vel < 20 && dist > en_vel
                            && abs(cur_angle) >= 20 && abs(cur_angle) <= 135;
            }

            if (game_.proto_map_external_id == 6 && turn_idx_ < 1175) {
                const double tg_angle = 51 * world.my_mirror().x;
                const double w1 = inverted ? 10 * w : w;
                const double w2 = inverted ? w : 10 * w;
                if (cur_angle + w1 < tg_angle) {
                    ret = inverted ? Action::LEFT : Action::RIGHT;
                } else if (cur_angle + w2 > tg_angle) {
                    ret = inverted ? Action::RIGHT : Action::LEFT;
                } else {
                    ret = Action::STOP;
                }
            } else if (h_control) {
                if (game_.proto_map_external_id == 1 && ((!inverted && w < -1) || (inverted && w > 1))) {
                    ret = Action::RIGHT;
                } else if (cur_angle + dw_2 * w > still_angle) {
                    ret = inverted ? Action::RIGHT : Action::LEFT;
                } else if (cur_angle + dw_1 * w < still_angle) {
                    ret = inverted ? Action::LEFT : Action::RIGHT;
                } else {
                    ret = Action::STOP;
                }
            }
        }
    }

    if (ret == Action::UNKNOWN && game_.proto_car.external_id == 1 && game_.proto_map_external_id == 6) {
        if (turn_idx_ < 35) {
            ret = Action::RIGHT;
        } else {
            const vec2 my_center = cpBodyLocalToWorld(world.me().body, cpBodyGetCenterOfGravity(world.me().body));
            const vec2 en_center = cpBodyLocalToWorld(world.enemy().body, cpBodyGetCenterOfGravity(world.enemy().body));
            const double cur_angle = to_deg(constraint_angle(cpBodyGetAngle(world.me().body)));

            bool h_control = (inverted && en_center.x < 600) || (!inverted && en_center.x > 600);

            if (h_control && turn_idx_ >= 1062) {
                ret = Action::LEFT;
            } else if (world.me().in_air && h_control) {
                const double angular_speed = cpBodyGetAngularVelocity(world.me().body);

                const double w = to_deg(angular_speed) * 0.016;

                const double tg_angle = 60 * world.my_mirror().x;
                const double w1 = inverted ? 10 * w : w;
                const double w2 = inverted ? w : 10 * w;
                if (cur_angle + w1 < tg_angle) {
                    ret = inverted ? Action::LEFT : Action::RIGHT;
                } else if (cur_angle + w2 > tg_angle) {
                    ret = inverted ? Action::RIGHT : Action::LEFT;
                } else {
                    ret = Action::STOP;
                }
            } else if (h_control) {
                vec2 my_vel = cpBodyGetVelocity(world.me().body);
                const double tg_x = inverted ? 950 : 250;
                if (my_center.x < tg_x && my_center.x + my_vel.x <= tg_x) {
                    ret = inverted ? Action::LEFT : Action::RIGHT;
                } else if (my_center.x > tg_x && my_center.x + my_vel.x >= tg_x) {
                    ret = inverted ? Action::RIGHT : Action::LEFT;
                } else {
                    ret = Action::STOP;
                }
            }
        }
    }

    if (ret != Action::UNKNOWN && ret != Action::STOP && world.my_mirror().x == -1) {
        ret = ret == Action::LEFT ? Action::RIGHT : Action::LEFT;
    }

    return ret;
}

std::string Strategy::debug_string() {
    char buf[100];
    sprintf(buf, "%e", sum_err_);
    return buf;
}

void Strategy::on_tick_start(const World &origin) {
    ++turn_idx_;
    VIS_MESSAGE("Me %d; Enemy %d; Turn %d\\n", game_.lives[0], game_.lives[1], turn_idx_)
    if (turn_idx_ == 0) {
        sim_.set_world(origin);
        cp::print_memory_usage();
    } else {
        sim_.step(turn_action_[0]);
    }
    if (turn_idx_ > 1) {
        ensure_perfect_simulation(origin);
    }

    if (!evaluator_) {
        evaluator_ = std::make_unique<Evaluator>(game_, sim_);
    }
}

Action Strategy::on_tick_end(Action action, const World &origin) {
    draw_game(game_, 1);
    sim_.draw();
    VIS_END_FRAME();

    //Advance turn
    std::swap(turn_dump_[0], turn_dump_[1]);
    std::swap(turn_action_[0], turn_action_[1]);
    std::swap(enemy_wheel_angle_[0], enemy_wheel_angle_[1]);

    //Save in previous turn
    sim_.save(turn_dump_[0]);
    turn_action_[0] = action;
    enemy_wheel_angle_[0] = get_drive_wheel_angle(origin);

    return action;
}

void Strategy::ensure_perfect_simulation(const World &world) {
    const double now_angle = get_drive_wheel_angle(world);
    const double predicted_angle = get_drive_wheel_angle(sim_.get_world());
    if (!eps_eq(now_angle, predicted_angle)) {
        //Ok, it was not stop action 2 ticks before
        Action enemy_act = now_angle > predicted_angle ? Action::LEFT : Action::RIGHT;
        sim_.restore(turn_dump_[1]);
        sim_.step(turn_action_[1], enemy_act);
        sim_.save(turn_dump_[0]);
        sim_.step(turn_action_[0]);
        VIS_MESSAGE("enemy action %s\\n", enemy_act == Action::RIGHT ? "RIGHT" : "LEFT");
    } else {
        VIS_MESSAGE("enemy action %s\\n", "STOP");
    }
}

double Strategy::get_drive_wheel_angle(const World &w) const {
    const auto &enemy = w.cars[1 - w.my_id];
    bool is_rear_drive = game_.proto_car.drive == ProtoCar::FR;
    return is_rear_drive ? enemy.rear_wheel.angle : enemy.front_wheel.angle;
}

void Strategy::sim_precision_checker(const World &world) {
    const auto &sim_world = sim_.get_world();
    for (int i = 0; i < 2; ++i) {
        static const auto wrap = [](double val) {
            return eps_eq(val, 0.0) ? 0.0 : val;
        };

        vec2 body_diff = sim_world.cars[i].body.origin - world.cars[i].body.origin;
        double angle_diff = sim_world.cars[i].body.angle - world.cars[i].body.angle;

        double r_wh_diff = sim_world.cars[i].rear_wheel.angle - world.cars[i].rear_wheel.angle;
        double f_wh_diff = sim_world.cars[i].front_wheel.angle - world.cars[i].front_wheel.angle;
        VIS_MESSAGE("Car_%d: (%6.6f, %6.6f, %6.6f)\\n", i, wrap(body_diff.x), wrap(body_diff.y), wrap(angle_diff));
        VIS_MESSAGE("r_wh = %.6f, f_wh = %.6f\\n", to_deg(r_wh_diff), to_deg(f_wh_diff));
        VIS_MESSAGE("%s\\n", sim_.check_in_air(i) ? "AIR" : "GROUND");

        sum_err_ += std::abs(body_diff.x) + std::abs(body_diff.y)
                    + std::abs(angle_diff) + std::abs(r_wh_diff) + std::abs(f_wh_diff);
    }
    VIS_MESSAGE("\\nerr: %e\\n", sum_err_);
    if (sum_err_ > 0.0) {
        LOG_WARN("Simulation error not zero: %e", sum_err_);
    }
}
