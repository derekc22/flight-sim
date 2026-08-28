#pragma once
#include <nlohmann/json.hpp>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& dyn_json);
    aerodynamics::ControlDerivatives parse_control_derivatives(const nlohmann::json& ctrl_json);
    void validate_surfaces_json(const nlohmann::json& surfaces_json);
    aerodynamics::AerodynamicsManager parse_aerodynamics_manager(const nlohmann::json& config);
}
