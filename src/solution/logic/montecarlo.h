//
// Created by valdemar on 09.09.18.
//

#pragma once

#include "../structures.h"
#include "../simulation/simulator.h"

#include <cstdint>
#include <optional>

namespace montecarlo {

class Genome {
public:
    ///Count of actions in genome
    static constexpr uint8_t DEPTH = 12;
    ///Each action should lasts TURN_LEN turns
    static constexpr uint8_t TURN_LEN = 5;

    Genome(const Evaluator *evaluator, bool with_random);

    void duplicate(Genome &other);

    void shift();

    void mutate();

    void mutate(Genome *other) const;

    ///Simulator should be in normal state
    double get_score(Simulator &sim, cp::transaction_t &mut_buffer, const Genome &enemy);

    Action get_action() const;

public:
    void randomize();

    void randomize(int sample_depth);

    Action actions_[DEPTH];
    double score_ = -1;
    bool scored_ = false;
    int small_shift_ = 0;

    const Evaluator *evaluator_;
};

} // namespace montecarlo