#include <nlohmann/json.hpp>
#include "core/json/sensors/private/parsing.hpp"
#include "core/json/sensors/public/parser.hpp"
#include "core/json/public/files.hpp"
#include "simulation/sensors/public/manager.hpp"

namespace json {

    sensors::SensorManager parse_sensors_config() {
        const auto config_path = resolve_run_config_entry_path("sensors_config");
        const auto config = read_json_file(config_path);
        return parse_sensor_manager(config);
    }

}
