#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/util/public.hpp"

namespace json {

    template <typename SurfaceActuatorType>
    SurfaceActuatorType parse_surface_actuator(const nlohmann::json& config, const std::string& key) {
        const auto& actuator_json = config.at(key);
        validate_actuator_json(actuator_json);

        return SurfaceActuatorType(
            util::deg_to_rad(actuator_json.at("limit_max").get<double>()),
            util::deg_to_rad(actuator_json.at("limit_min").get<double>()),
            actuator_json.at("tau").get<double>()
        );
    }

    template <typename PropulsorActuatorType>
    PropulsorActuatorType parse_propulsor_actuator(const nlohmann::json& config, const std::string& key, structural::StructuralProperties& structural_properties) {
        const auto& actuator_json = config.at(key);
        validate_propulsor_actuator_json(actuator_json, key);

        PropulsorActuatorType propulsor(
            actuator_json.at("limit_max").get<double>(),
            actuator_json.at("limit_min").get<double>(),
            actuator_json.at("tau").get<double>(),
            util::deg_to_rad(actuator_json.at("inclination_angle").get<double>()),
            util::deg_to_rad(actuator_json.at("toe_angle").get<double>()),
            parse_Vector3d(actuator_json.at("pB_prop_cg"))
        );

        if (actuator_json.contains("propellers")) {
            propulsor.propellers = parse_propellers(actuator_json.at("propellers"), structural_properties, propulsor.n_prop);
        }

        return propulsor;
    }
}
