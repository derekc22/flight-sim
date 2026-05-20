#pragma once
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/actuators/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    void validate_actuator_json(const nlohmann::json& actuator_json);
    void validate_propulsor_actuator_json(const nlohmann::json& propulsor_actuator_json, const std::string& key);
    void validate_propellers_json(const nlohmann::json& propellers_json);
    actuators::PropellerAssembly parse_propellers(const nlohmann::json& propellers_json, structural::StructuralProperties& structural_properties, const Eigen::Vector3d& n_prop);

    template <typename SurfaceActuatorType>
    SurfaceActuatorType parse_surface_actuator(const nlohmann::json& config, const std::string& key);

    template <typename PropulsorActuatorType>
    PropulsorActuatorType parse_propulsor_actuator(const nlohmann::json& config, const std::string& key, structural::StructuralProperties& structural_properties);

    actuators::ActuatorProperties parse_actuator_properties(const nlohmann::json& config, structural::StructuralProperties& structural_properties);
}

#include "core/json/actuators/private.tpp"
