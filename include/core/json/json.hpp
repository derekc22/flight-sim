#pragma once
#include <string>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/control/control.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "simulation/avionics/avionics.hpp"

namespace json {

    void dump_configs(const std::string& dir);
    aerodynamics::AerodynamicProperties parse_aerodynamics_config();
    control::ControlProperties parse_control_config();
    vehicles::StepOptions parse_initialization_config(bool trim_enabled);
    structural::StructuralProperties parse_structural_config();
    avionics::AvionicsProperties parse_avionics_config();

}
