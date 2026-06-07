#include <nlohmann/json.hpp>
#include "core/json/runtime/private.hpp"
#include "core/json/runtime/public.hpp"
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/runtime/public.hpp"

namespace json {

    runtime::RuntimeProperties parse_runtime_config(const actuators::ActuatorProperties& actuator_properties) {
        const auto config_path = resolve_run_config_entry_path("runtime_config");
        const auto config = read_json_file(config_path);
        const runtime::RuntimeProperties runtime_properties = parse_runtime_properties(config, actuator_properties);
        return runtime_properties;
    }

}
