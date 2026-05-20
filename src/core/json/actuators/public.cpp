#include <nlohmann/json.hpp>
#include "core/json/actuators/private.hpp"
#include "core/json/actuators/public.hpp"
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    actuators::ActuatorProperties parse_actuator_config(structural::StructuralProperties& structural_properties) {
        const auto config_path = resolve_run_config_entry_path("actuator_config");
        const auto config = read_json_file(config_path);
        return parse_actuator_properties(config, structural_properties);
    }

}
