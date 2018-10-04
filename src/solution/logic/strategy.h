//
// Created by valdemar on 01.09.18.
//

#pragma once

#include "../simulation/simulator.h"
#include "../structures.h"
#include "montecarlo.h"
#include "evaluator.h"

#include <chrono>

class Strategy {
public:
    ~Strategy();

    void next_match(Game game);

    Action move(World world);

    std::string debug_string();

private:
    Action heuristics_control();

    void on_tick_start(const World &origin);

    ///Should be issued on turn end
    Action on_tick_end(Action action, const World &origin);

    ///Predict opponent move on previous turn
    void ensure_perfect_simulation(const World &world);

    double get_drive_wheel_angle(const World &w) const;

    void sim_precision_checker(const World &world);

    //Dumps to preserve simulation precision
    //0 - previous turn
    //1 - turn before previous
    cp::transaction_t turn_dump_[2];
    Action turn_action_[2];
    double enemy_wheel_angle_[2];

    ///For various simulations
    cp::transaction_t active_buf_;

    Game game_;
    Simulator sim_;
    int turn_idx_ = -1;

    double sum_err_ = 0.0;

    std::unique_ptr<montecarlo::Genome> solution_;
    std::unique_ptr<montecarlo::Genome> enemy_solution_;
    std::unique_ptr<Evaluator> evaluator_;
};