#pragma once
#include <Eigen/Dense>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    template <typename SurfaceActuatorType>
    SurfaceActuatorType parse_surface_actuator(const nlohmann::json& config, const std::string& key) {
        if (config.at(key).is_null()) {
            SurfaceActuatorType null_surface(
                0.0, 0.0, 0.0
            );
            return null_surface;
        }

        const auto& surface_actuator_json = config.at(key);
        validate_actuator_json(surface_actuator_json);

        return SurfaceActuatorType(
            surface_actuator_json.at("limit_max").get<double>(),
            surface_actuator_json.at("limit_min").get<double>(),
            surface_actuator_json.at("tau").get<double>()
        );
    }

    template <typename PropulsorActuatorType>
    PropulsorActuatorType parse_propulsor_actuator(const nlohmann::json& config, const std::string& key, structural::StructuralProperties& structural_properties) {
        if (config.at(key).is_null()) {
            PropulsorActuatorType null_propulsor(
                0.0, 0.0, 0.0, 0.0, 0.0,
                Eigen::Vector3d(0, 0, 0)
            );
            return null_propulsor;
        }

        const auto& propulsor_actuator_json = config.at(key);
        validate_propulsor_actuator_json(propulsor_actuator_json, key);

        std::string geometry_id = propulsor_actuator_json.at("geometry_id").get<std::string>();
        const structural::Geometry& geom = structural_properties.get_geometry(geometry_id);
        validate_propulsor_actuator_placement(geom.p_ref, key);
        Eigen::Vector3d p_propulsor_cg = geom.p_ref - structural_properties.p_cg.data;

        PropulsorActuatorType propulsor(
            propulsor_actuator_json.at("limit_max").get<double>(),
            propulsor_actuator_json.at("limit_min").get<double>(),
            propulsor_actuator_json.at("tau").get<double>(),
            propulsor_actuator_json.at("inclination_angle").get<double>(),
            propulsor_actuator_json.at("toe_angle").get<double>(),
            p_propulsor_cg
        );

        if (propulsor_actuator_json.contains("propellers")) {
            propulsor.propellers = parse_propellers(propulsor_actuator_json.at("propellers"), structural_properties, propulsor.n_prop);
        }

        return propulsor;
    }
}
