//
// Created by valdemar on 02.09.18.
//

#pragma once

#include "../structures.h"
#include "cp_helpers.h"

#include <chipmunk/chipmunk.h>
#include <chipmunk/chipmunk_structs.h>

class Evaluator;

struct NativeWorld {
    struct CarDesc {
        cpBody *body;
        cpShape *button;
        bool loosed;
        bool in_air;
    };

    const CarDesc &me() const {
        return cars[my_id];
    }

    const CarDesc &enemy() const {
        return cars[1 - my_id];
    }

    const vec2 my_mirror() const {
        return mirror_[my_id];
    }

    const vec2 en_mirror() const {
        return mirror_[1 - my_id];
    }

    CarDesc cars[2];
    int my_id;
    vec2 mirror_[2];
    int turn_idx;
};

class Simulator {
    static constexpr size_t TICK_TO_DEADLINE = 600;
public:
    friend class Evaluator;

    void init(const Game *game);

    void set_world(const World &world);

    void step(Action my_action = Action::STOP, Action enemy_action = Action::STOP);

    void draw() const;

    const World &get_world() const;
    const NativeWorld &world_native(bool as_enemy = false) const;

    bool check_in_air(int car_idx) const;

    void save(cp::transaction_t &to);
    //void fast_save();

    void restore(const cp::transaction_t &from);
    //void fast_restore();

    ///Swap me and enemy for action simulation
    void swap_sides();

private:
    struct cp_wheel_t {
        cpBody *body;
        cpShape *shape;
        cpConstraint *joint;
        cpConstraint *damp;
        cpConstraint *motor;
    };

    struct cp_car_t {
        cpBody *body;
        cpShape *shape;
        cpShape *button;
        cpShapeFilter filter;
        cp_wheel_t front_wheel;
        cp_wheel_t rear_wheel;
    };

    struct cp_deadline_t {
        cpShape *shape;
        cpBody *body = nullptr;
    };

    cp_car_t create_car(const CarDescription &car, cpGroup car_group);

    Simulator::cp_wheel_t create_wheel(const ProtoCar::wheel_t &proto,
                                       const cp_car_t &car,
                                       bool squared_wheels,
                                       int x_modification,
                                       bool create_motor);

    void car_apply_action(int idx, Action action);

    bool car_in_air(const cp_car_t &car) const;

    void update_world() const;

    const Game *game_ = nullptr;
    mutable World cur_w;
    mutable bool world_changed_ = false;
    mutable NativeWorld cur_w_native_;
    int my_real_id_ = 0;

    cp::Space space_;
    cp_deadline_t deadline_;
    uint16_t ticks_to_deadline_;
    cp_car_t cars_[2];

    std::vector<cpBody> body_values_;
    double saved_deadline_mark_;
    uint16_t saved_ticks_to_deadline_;
    size_t full_save_ticks_to_deadline_;

    cpCollisionHandler *handler_[2];
};