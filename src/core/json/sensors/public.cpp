#include <nlohmann/json.hpp>
#include "core/json/sensors/private.hpp"
#include "core/json/sensors/public.hpp"
#include "core/json/public.hpp"
#include "simulation/sensors/public.hpp"

namespace json {

    sensors::SensorProperties parse_sensors_config() {
        const auto config_path = resolve_run_config_entry_path("sensors_config");
        const auto config = read_json_file(config_path);
        return parse_sensor_properties(config);
    }

}
