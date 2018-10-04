//
// Created by valdemar on 02.09.18.
//

#include "simulator.h"
#include "../common/vis_debug.h"

#include <chipmunk/chipmunk.h>

cpBool callback_0(cpArbiter *, cpSpace *, cpDataPointer user_data) {
    static_cast<NativeWorld *>(user_data)->cars[0].loosed = true;
    return 1;
};

cpBool callback_1(cpArbiter *, cpSpace *, cpDataPointer user_data) {
    static_cast<NativeWorld *>(user_data)->cars[1].loosed = true;
    return 1;
};

void Simulator::init(const Game *game) {
    game_ = game;

    space_.clear();
    cpSpaceSetGravity(space_.native(), {0.0, -700.0});
    cpSpaceSetDamping(space_.native(), 0.85);

    handler_[0] = cpSpaceAddWildcardHandler(space_.native(), 20);
    handler_[1] = cpSpaceAddWildcardHandler(space_.native(), 30);

    handler_[0]->userData = &cur_w_native_;
    handler_[0]->beginFunc = callback_0;

    handler_[1]->userData = &cur_w_native_;
    handler_[1]->beginFunc = callback_1;

    //Map box
    const double segment_height = 10;
    const double bo = segment_height - 1;
    const double max_height = 800.0;
    const double max_width = 1200.0;
    static const std::vector<std::pair<vec2, vec2>> boundaries = {
        {{-bo,            -bo},             {-bo,            max_height + bo}},
        {{-bo,            max_height + bo}, {max_width + bo, max_height + bo}},
        {{max_width + bo, max_height + bo}, {max_width + bo, -bo}},
        {{max_width + bo, -bo},             {-bo,            -bo}}
    };
    for (auto[b0, b1] : boundaries) {
        auto shape = cpSegmentShapeNew(space_.static_body(), b0, b1, segment_height);
        cpShapeSetSensor(shape, 1);
        space_.add_shape(shape);
    }

    //Create map
    for (const auto &seg : game_->proto_map) {
        auto segment = cpSegmentShapeNew(space_.static_body(), seg.p1, seg.p2, seg.height);
        cpShapeSetFriction(segment, 1.0);
        cpShapeSetElasticity(segment, 0.0);
        space_.add_shape(segment);
    }

    //Add deadline
    ticks_to_deadline_ = TICK_TO_DEADLINE;
    if (deadline_.body) {
        cpBodyFree(deadline_.body);
    }
    deadline_.body = cpBodyNewKinematic();
    const cpVect deadline_pts[] = {
        {0.0, 2.0}, {1200.0, 2.0}, {1200.0, -20.0}, {0.0, -20.0}
    };
    deadline_.shape = cpPolyShapeNew(deadline_.body, 4, deadline_pts, cpTransform{1, 0, 0, 1, 0, 0}, 0);
    cpShapeSetSensor(deadline_.shape, 1);
    cpBodySetPosition(deadline_.body, {0.0, 10.0});
    space_.add_shape(deadline_.shape);
}

void Simulator::set_world(const World &world) {
    cur_w = world;

    cpGroup car_groups[] = {2, 3};
    for (int i = 0; i < 2; ++i) {
        cars_[i] = create_car(cur_w.cars[i], car_groups[i]);
        space_.add_shape(cars_[i].button);
        space_.add_body(cars_[i].body);
        space_.add_shape(cars_[i].shape);
        space_.add_body(cars_[i].rear_wheel.body);
        space_.add_body(cars_[i].front_wheel.body);

        {
            const auto &wheel = cars_[i].rear_wheel;
            space_.add_shape(wheel.shape);
            space_.add_constraint(wheel.joint);
            space_.add_constraint(wheel.damp);
        }

        {
            const auto &wheel = cars_[i].front_wheel;
            space_.add_shape(wheel.shape);
            space_.add_constraint(wheel.joint);
            space_.add_constraint(wheel.damp);
        }

        if (cars_[i].rear_wheel.motor) {
            space_.add_constraint(cars_[i].rear_wheel.motor);
        }
        if (cars_[i].front_wheel.motor) {
            space_.add_constraint(cars_[i].front_wheel.motor);
        }
    }

    //Init native world
    my_real_id_ = world.my_id;
    cur_w_native_.cars[0] = {cars_[0].body, cars_[0].button, false, check_in_air(0)};
    cur_w_native_.cars[1] = {cars_[1].body, cars_[1].button, false, check_in_air(1)};
    cur_w_native_.mirror_[world.my_id] = vec2(world.me().x_modification, 1);
    cur_w_native_.mirror_[1 - world.my_id] = vec2(world.enemy().x_modification, 1);
    cur_w_native_.turn_idx = 0;
}

void Simulator::step(Action my_action, Action enemy_action) {
    static constexpr double dt = 0.016;

    car_apply_action(my_real_id_, my_action);
    car_apply_action(1 - my_real_id_, enemy_action);

    if (ticks_to_deadline_ < 1) {
        auto deadline_pos = cpBodyGetPosition(deadline_.body);
        cpBodySetPosition(deadline_.body, deadline_pos + vec2{0, 0.5});
    } else {
        --ticks_to_deadline_;
    }

    cpSpaceStep(space_.native(), dt);

    cur_w_native_.cars[0].in_air = check_in_air(0);
    cur_w_native_.cars[1].in_air = check_in_air(1);
    ++cur_w_native_.turn_idx;

    world_changed_ = true;
}

void Simulator::draw() const {
    update_world();
#ifdef ENABLE_VISUALISER
    draw_world(*game_, cur_w);
#endif
}

const World &Simulator::get_world() const {
    update_world();
    return cur_w;
}

const NativeWorld &Simulator::world_native(bool as_enemy) const {
    cur_w_native_.my_id = as_enemy ? 1 - my_real_id_ : my_real_id_;
    return cur_w_native_;
}

bool Simulator::check_in_air(int car_idx) const {
    assert(car_idx == 0 || car_idx == 1);
    return car_in_air(cars_[car_idx]);
}

void Simulator::save(cp::transaction_t &to) {
    auto bytes = cp::memdump(to.ptr.get());
    to.bytes = bytes;
    to.ticks_to_deadline = ticks_to_deadline_;
    for (int i = 0; i < 2; ++i) {
        to.loose_state[i] = cur_w_native_.cars[i].loosed;
        to.in_air_state[i] = cur_w_native_.cars[i].in_air;
    }
    to.turn_idx = cur_w_native_.turn_idx;
}

void Simulator::restore(const cp::transaction_t &from) {
    //TODO: It looks like deadline position restoration works wrong
    cp::memload(from.ptr.get(), from.bytes);
    ticks_to_deadline_ = from.ticks_to_deadline;
    cur_w.deadline_mark = cpBodyGetPosition(deadline_.body).y;
    for (int i = 0; i < 2; ++i) {
        cur_w_native_.cars[i].loosed = from.loose_state[i];
        cur_w_native_.cars[i].in_air = from.in_air_state[i];
    }
    cur_w_native_.turn_idx = from.turn_idx;
    world_changed_ = true;
}

void Simulator::swap_sides() {
    my_real_id_ = 1 - my_real_id_;
}

Simulator::cp_car_t Simulator::create_car(const CarDescription &car, cpGroup car_group) {
    //Create car
    cp_car_t ret;
    const auto &proto_car = game_->proto_car;

    std::vector<cpVect> processed_pts;
    processed_pts.reserve(proto_car.body_poly.size());
    for (auto &p : proto_car.body_poly) {
        processed_pts.push_back({p.x * car.x_modification, p.y});
    }

    double car_moment = cpMomentForPoly(proto_car.body_mass, static_cast<int>(processed_pts.size()),
                                        processed_pts.data(), {0.0, 0.0}, 0.0);

    ret.body = cpBodyNew(proto_car.body_mass, car_moment);

    ret.shape = cpPolyShapeNew(ret.body, static_cast<int>(processed_pts.size()), processed_pts.data(),
                               cpTransform{1, 0, 0, 1, 0, 0}, 0);
    cpShapeSetFriction(ret.shape, proto_car.body_friction);
    cpShapeSetElasticity(ret.shape, proto_car.body_elasticity);
    ret.filter = cpShapeFilterNew(car_group, 0xffffffff, 0xffffffff);
    cpShapeSetFilter(ret.shape, ret.filter);

    std::vector<cpVect> button_pts;
    button_pts.reserve(proto_car.button_poly.size());
    for (const auto &p : proto_car.button_poly) {
        button_pts.push_back({p.x * car.x_modification, p.y});
    }

    ret.button = cpPolyShapeNew(ret.body, static_cast<int>(button_pts.size()), button_pts.data(),
                                cpTransform{1, 0, 0, 1, 0, 0}, 0);
    cpShapeSetFilter(ret.button, ret.filter);
    cpShapeSetSensor(ret.button, 1);
    cpShapeSetCollisionType(ret.button, car_group * 10);

    cpBodySetCenterOfGravity(ret.body, cpShapeGetCenterOfGravity(ret.shape));

    //Wheels
    ret.front_wheel = create_wheel(proto_car.front_wheel, ret, proto_car.squared_wheels, car.x_modification,
                                   proto_car.drive == ProtoCar::AWD || proto_car.drive == ProtoCar::FF);

    ret.rear_wheel = create_wheel(proto_car.rear_wheel, ret, proto_car.squared_wheels, car.x_modification,
                                  proto_car.drive == ProtoCar::AWD || proto_car.drive == ProtoCar::FR);

    //Set position
    cpBodySetPosition(ret.body, car.body.origin);
    cpBodySetPosition(
        ret.front_wheel.body,
        car.body.origin + vec2{proto_car.front_wheel.position.x * car.x_modification, proto_car.front_wheel.position.y}
    );
    cpBodySetPosition(
        ret.rear_wheel.body,
        car.body.origin + vec2{proto_car.rear_wheel.position.x * car.x_modification, proto_car.rear_wheel.position.y}
    );

    return ret;
}

Simulator::cp_wheel_t Simulator::create_wheel(const ProtoCar::wheel_t &proto, const cp_car_t &car,
                                              bool squared_wheels, int x_modification, bool create_motor) {
    cp_wheel_t wheel;

    double moment;
    if (squared_wheels) {
        moment = cpMomentForBox(proto.mass, proto.radius * 2.0, proto.radius * 2.0);
    } else {
        moment = cpMomentForCircle(proto.mass, 0.0, proto.radius, {0, 0});
    }
    wheel.body = cpBodyNew(proto.mass, moment);
    cpBodySetPosition(wheel.body, {proto.position.x * x_modification, proto.position.y});

    if (squared_wheels) {
        wheel.shape = cpBoxShapeNew(wheel.body, proto.radius * 2.0, proto.radius * 2.0, 0);
    } else {
        wheel.shape = cpCircleShapeNew(wheel.body, proto.radius, {0, 0});
    }
    cpShapeSetFilter(wheel.shape, car.filter);
    cpShapeSetFriction(wheel.shape, proto.friction);
    cpShapeSetElasticity(wheel.shape, proto.elasticity);

    wheel.joint = cpGrooveJointNew(
        car.body, wheel.body,
        {proto.damp_position.x * x_modification, proto.damp_position.y - proto.groove_offset},
        {proto.damp_position.x * x_modification, proto.damp_position.y - proto.damp_length * 1.5},
        {0.0, 0.0}
    );

    wheel.damp = cpDampedSpringNew(
        wheel.body, car.body,
        {0, 0}, {proto.damp_position.x * x_modification, proto.damp_position.y},
        proto.damp_length, proto.damp_stiffness, proto.damp_damping
    );

    wheel.motor = nullptr;
    if (create_motor) {
        wheel.motor = cpSimpleMotorNew(wheel.body, car.body, 0);
    }

    return wheel;
}

void Simulator::car_apply_action(int idx, Action action) {
    cp_car_t &car = cars_[idx];

    double rate;
    if (action == Action::STOP) {
        rate = 0.0;
    } else {
        rate = game_->proto_car.max_speed * (action == Action::RIGHT ? -1.0 : 1.0);
    }

    //In air
    if (action != Action::STOP && cur_w_native_.cars[idx].in_air) {
        double torque = game_->proto_car.torque * (action == Action::LEFT ? -1.0 : 1.0);
        cpBodySetTorque(car.body, torque);
    }

    //On the ground
    if (car.rear_wheel.motor) {
        cpSimpleMotorSetRate(car.rear_wheel.motor, rate);
    }
    if (car.front_wheel.motor) {
        cpSimpleMotorSetRate(car.front_wheel.motor, rate);
    }
}

bool Simulator::car_in_air(const Simulator::cp_car_t &car) const {
    const cpShape *rear_wheel_touch = cpSpacePointQueryNearest(
        space_.native(), cpBodyGetPosition(car.rear_wheel.body),
        game_->proto_car.rear_wheel.radius + 1, car.filter, nullptr
    );

    if (rear_wheel_touch) {
        return false;
    }

    const cpShape *front_wheel_touch = cpSpacePointQueryNearest(
        space_.native(), cpBodyGetPosition(car.front_wheel.body),
        game_->proto_car.front_wheel.radius + 1, car.filter, nullptr
    );

    return front_wheel_touch == nullptr;
}

void Simulator::update_world() const {
    if (!world_changed_) {
        return;
    }
    world_changed_ = false;

    if (ticks_to_deadline_ < 1) {
        cur_w.deadline_mark = cpBodyGetPosition(deadline_.body).y;
    }

    for (int i = 0; i < 2; ++i) {
        auto &car = cur_w.cars[i];
        car.body.origin = cpBodyGetPosition(cars_[i].body);
        car.body.angle = cpBodyGetAngle(cars_[i].body);
        car.front_wheel.origin = cpBodyGetPosition(cars_[i].front_wheel.body);
        car.front_wheel.angle = cpBodyGetAngle(cars_[i].front_wheel.body);
        car.rear_wheel.origin = cpBodyGetPosition(cars_[i].rear_wheel.body);
        car.rear_wheel.angle = cpBodyGetAngle(cars_[i].rear_wheel.body);
    }
}
