#pragma once
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    void validate_actuator_json(const nlohmann::json& actuator_json);

    void validate_propulsor_actuator_json(const nlohmann::json& propulsor_actuator_json);

    void validate_propulsor_actuator_placement(const Eigen::Vector3d& p_propulsor, const std::string& key);

    void validate_propellers_json(const nlohmann::json& propellers_json);

    actuators::PropellerAssembly parse_propellers(const nlohmann::json& propellers_json, structural::StructuralManager& structural_manager, const Eigen::Vector3d& n_prop);

    template <typename SurfaceActuatorType>
    SurfaceActuatorType parse_surface_actuator(const nlohmann::json& config, const std::string& key);

    template <typename PropulsorActuatorType>
    PropulsorActuatorType parse_propulsor_actuator(const nlohmann::json& config, const std::string& key, structural::StructuralManager& structural_manager);

    actuators::ActuatorManager parse_actuator_manager(const nlohmann::json& config, structural::StructuralManager& structural_manager);
}

#include "core/json/actuators/private.tpp"
