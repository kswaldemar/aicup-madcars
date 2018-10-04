//
// Created by valdemar on 31.08.18.
//

#include "structures.h"

void from_json(const nlohmann::json &j, vec2 &obj) {
    obj.x = j[0].get<double>();
    obj.y = j[1].get<double>();
}

void from_json(const nlohmann::json &j, AnglePoint &obj) {
    obj.origin = {j[0].get<double>(), j[1].get<double>()};
    obj.angle = j[2].get<double>();
}

void from_json(const nlohmann::json &j, CarDescription &obj) {
    obj.body.origin = {j[0][0].get<double>(), j[0][1].get<double>()};
    obj.body.angle = j[1].get<double>();
    obj.x_modification = j[2].get<int>();
    obj.rear_wheel = j[3].get<AnglePoint>();
    obj.front_wheel = j[4].get<AnglePoint>();
}

void from_json(const nlohmann::json &j, World &obj) {
    obj.cars[0] = j["my_car"].get<CarDescription>();
    obj.cars[1] = j["enemy_car"].get<CarDescription>();
    if (obj.cars[0].x_modification == -1) {
        //Our strategy on right side
        std::swap(obj.cars[0], obj.cars[1]);
        obj.my_id = 1;
    }
    obj.loosed[0] = false;
    obj.loosed[1] = false;
    obj.deadline_mark = j["deadline_position"].get<double>();
}

void from_json(const nlohmann::json &j, Segment &obj) {
    obj.p1 = {j[0][0].get<double>(), j[0][1].get<double>()};
    obj.p2 = {j[1][0].get<double>(), j[1][1].get<double>()};
    obj.height = j[2].get<double>();
}

void from_json(const nlohmann::json &j, ProtoCar &obj) {
    obj.body_poly = j["car_body_poly"].get<std::vector<vec2>>();
    obj.button_poly = j["button_poly"].get<std::vector<vec2>>();
    if (auto it = j.find("squared_wheels"); it != j.end()) {
        obj.squared_wheels = static_cast<bool>(it->get<int>());
    } else {
        obj.squared_wheels = false;
    }

    const auto read_wheel = [&j](std::string prefix, ProtoCar::wheel_t &wheel) {
        //@formatter:off
        wheel.damp_damping   = j[prefix + "wheel_damp_damping"].get<double>();
        wheel.damp_length    = j[prefix + "wheel_damp_length"].get<double>();
        wheel.damp_position  = j[prefix + "wheel_damp_position"].get<vec2>();
        wheel.damp_stiffness = j[prefix + "wheel_damp_stiffness"].get<double>();
        wheel.elasticity     = j[prefix + "wheel_elasticity"].get<double>();
        wheel.friction       = j[prefix + "wheel_friction"].get<double>();
        wheel.groove_offset  = j[prefix + "wheel_groove_offset"].get<double>();
        wheel.mass           = j[prefix + "wheel_mass"].get<double>();
        wheel.radius         = j[prefix + "wheel_radius"].get<double>();
        wheel.position       = j[prefix + "wheel_position"].get<vec2>();
        //@formatter:on
    };

    read_wheel("front_", obj.front_wheel);
    read_wheel("rear_", obj.rear_wheel);

    obj.drive = static_cast<ProtoCar::DriveType>(j["drive"].get<int>());
    obj.body_elasticity = j["car_body_elasticity"].get<double>();
    obj.body_friction = j["car_body_friction"].get<double>();
    obj.body_mass = j["car_body_mass"].get<double>();
    obj.external_id = j["external_id"].get<int>();

    obj.max_angular_speed = j["max_angular_speed"].get<double>();
    obj.max_speed = j["max_speed"].get<double>();
    obj.torque = j["torque"].get<double>();
}

void from_json(const nlohmann::json &j, Game &obj) {
    obj.lives[0] = j["my_lives"].get<int>();
    obj.lives[1] = j["enemy_lives"].get<int>();
    obj.proto_map = j["proto_map"]["segments"].get<std::vector<Segment>>();
    obj.proto_map_external_id = j["proto_map"]["external_id"];
    obj.proto_car = j["proto_car"].get<ProtoCar>();
}

std::string action_to_command(Action obj, const std::string &debug) {
    std::string ret = [obj]{
        assert(obj != Action::UNKNOWN);
        switch (obj) {
            case Action::LEFT: return  "{\"command\": \"left\"";
            case Action::RIGHT: return "{\"command\": \"right\"";
            case Action::STOP: return  "{\"command\": \"stop\"";
            default:
                return "No way";
        }
        return "";
    }();
    if (!debug.empty()) {
         ret += ", \"debug\": \"" + debug + "\"";
    }
    ret += "}";
    return ret;
}

