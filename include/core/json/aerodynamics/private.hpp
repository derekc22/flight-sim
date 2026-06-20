#pragma once
#include <nlohmann/json.hpp>
#include "core/json/aerodynamics/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& dyn_json);
    aerodynamics::ControlDerivatives parse_control_derivatives(const nlohmann::json& ctrl_json);
    aerodynamics::AerodynamicProperties parse_aerodynamic_properties(const nlohmann::json& config, const structural::StructuralProperties& structural_properties);
}
