#include <nlohmann/json.hpp>
#include "core/json/operating/private.hpp"
#include "core/json/operating/public.hpp"
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/operating/public.hpp"

namespace json {

    operating::OperatingProperties parse_operating_config(const actuators::ActuatorProperties& actuator_properties) {
        const auto config_path = resolve_run_config_entry_path("operating_config");
        const auto config = read_json_file(config_path);
        const operating::OperatingProperties operating_properties = parse_operating_properties(config);
        validate_operating_properties(operating_properties, actuator_properties);
        return operating_properties;
    }

}
