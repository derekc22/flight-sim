#include <nlohmann/json.hpp>
#include "core/json/actuators/private/parsing.hpp"
#include "core/json/actuators/public/parser.hpp"
#include "core/json/public/files.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/structural/public/manager.hpp"

namespace json {

    actuators::ActuatorManager parse_actuator_config(structural::StructuralManager& structural_manager) {
        const auto config_path = resolve_run_config_entry_path("actuator_config");
        const auto config = read_json_file(config_path);
        return parse_actuator_manager(config, structural_manager);
    }

}
