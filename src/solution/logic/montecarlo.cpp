//
// Created by valdemar on 09.09.18.
//

#include "montecarlo.h"
#include "fastrand.h"
#include "evaluator.h"

namespace montecarlo {

Genome::Genome(const Evaluator *evaluator, bool with_random) {
    evaluator_ = evaluator;
    scored_ = false;

    if (with_random) {
        randomize();
    } else {
        for (int i = 0; i < DEPTH; ++i) {
            actions_[i] = Action::STOP;
        }
    }
}

void Genome::duplicate(Genome &other) {
    memcpy(other.actions_, actions_, DEPTH * sizeof(actions_[0]));
    other.score_ = score_;
    other.scored_ = scored_;
}

void Genome::shift() {
    ++small_shift_;
    if (small_shift_ == TURN_LEN) {
        for (size_t i = 1; i < DEPTH; ++i) {
            actions_[i - 1] = actions_[i];
        }
        small_shift_ = 0;
        randomize(DEPTH - 1);
    }
    scored_ = false;
}

void Genome::mutate() {
    randomize(rand_int(DEPTH));
}

void Genome::mutate(Genome *other) const {
    memcpy(other->actions_, actions_, DEPTH * sizeof(actions_[0]));
    other->mutate();
    other->scored_ = false;
}

double Genome::get_score(Simulator &sim, cp::transaction_t &mut_buffer, const Genome &enemy) {
    static const auto k_attenuation = [] {
        std::vector<double> ret;
        ret.reserve(DEPTH);
        double val = 1.0;
        for (int i = 1; i <= DEPTH * TURN_LEN; ++i) {
            val *= 0.98;
            if (i % TURN_LEN == 0) {
                ret.push_back(val);
            }
        }
        return ret;
    }();

    if (!scored_) {
        scored_ = true;
        score_ = 0.0;
        for (int i = small_shift_; i < TURN_LEN; ++i) {
            sim.step(actions_[0], enemy.actions_[0]);
        }
        score_ += k_attenuation[0] * evaluator_->eval(sim.world_native());
        if (!sim.world_native().cars[0].loosed && !sim.world_native().cars[1].loosed) {
            for (size_t act_idx = 1; act_idx < DEPTH; ++act_idx) {
                for (int i = 0; i < TURN_LEN; ++i) {
                    sim.step(actions_[act_idx], enemy.actions_[act_idx]);
                }
                score_ += k_attenuation[act_idx] * evaluator_->eval(sim.world_native());
                if (sim.world_native().cars[0].loosed || sim.world_native().cars[1].loosed) {
                    break;
                }
            }
        }
        //Restore world
        sim.restore(mut_buffer);
    }
    return score_;
}

Action Genome::get_action() const {
    return actions_[0];
}

void Genome::randomize() {
    for (int i = 0; i < DEPTH; ++i) {
        randomize(i);
    }
}

void Genome::randomize(int sample_depth) {
    actions_[sample_depth] = static_cast<Action>(rand_int(3));
}


} // namespace montecarlo