//
// Created by valdemar on 31.08.18.
//

#pragma once

#include "vec2.h"
#include "common/json.h"

enum class Action {
    LEFT,
    RIGHT,
    STOP,
    UNKNOWN
};

struct Segment {
    vec2 p1;
    vec2 p2;
    double height;
};

struct ProtoCar {
    enum DriveType {
        FF = 1,
        FR = 2,
        AWD = 3,
    };

    struct wheel_t {
        double damp_damping;
        double damp_length;
        vec2 damp_position;
        double damp_stiffness;
        double elasticity;
        double friction;
        double groove_offset;
        double mass;
        vec2 position;
        double radius;
    };

    std::vector<vec2> button_poly;
    std::vector<vec2> body_poly;
    DriveType drive;
    int external_id;
    double body_elasticity;
    double body_friction;
    double body_mass;
    double max_angular_speed;
    double max_speed;
    double torque;
    bool squared_wheels;

    wheel_t front_wheel;
    wheel_t rear_wheel;
};

struct Game {
    int lives[2];
    std::vector<Segment> proto_map;
    int proto_map_external_id;
    ProtoCar proto_car;
};

struct AnglePoint {
    vec2 origin;
    double angle;
};

struct CarDescription {
    AnglePoint body;
    AnglePoint rear_wheel;
    AnglePoint front_wheel;
    int x_modification;
};

struct World {
    CarDescription cars[2];
    double deadline_mark = 0.0;
    int my_id = 0;
    bool loosed[2];

    inline const CarDescription &me() const {
        return cars[my_id];
    }

    inline const CarDescription &enemy() const {
        return cars[1 - my_id];
    }
};

void from_json(const nlohmann::json &j, vec2 &obj);
void from_json(const nlohmann::json &j, AnglePoint &obj);
void from_json(const nlohmann::json &j, CarDescription &obj);
void from_json(const nlohmann::json &j, World &obj);
void from_json(const nlohmann::json &j, Segment &obj);
void from_json(const nlohmann::json &j, ProtoCar &obj);
void from_json(const nlohmann::json &j, Game &obj);

std::string action_to_command(Action obj, const std::string &debug = "");
