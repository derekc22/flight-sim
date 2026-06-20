#include <nlohmann/json.hpp>
#include "core/json/aerodynamics/private.hpp"
#include "core/json/aerodynamics/public.hpp"
#include "core/json/public.hpp"
#include "simulation/aerodynamics/public.hpp"

namespace json {

    aerodynamics::AerodynamicProperties parse_aerodynamics_config(const structural::StructuralProperties& structural_properties) {
        const auto config_path = resolve_run_config_entry_path("aerodynamics_config");
        const auto config = read_json_file(config_path);
        return parse_aerodynamic_properties(config, structural_properties);
    }

}
