#include <nlohmann/json.hpp>
#include "core/json/actuators/private.hpp"
#include "core/json/actuators/public.hpp"
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    actuators::ActuatorManager parse_actuator_config(structural::StructuralManager& structural_manager) {
        const auto config_path = resolve_run_config_entry_path("actuator_config");
        const auto config = read_json_file(config_path);
        return parse_actuator_manager(config, structural_manager);
    }

}
