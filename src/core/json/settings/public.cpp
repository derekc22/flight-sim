#include <nlohmann/json.hpp>
#include "core/json/settings/private.hpp"
#include "core/json/settings/public.hpp"
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/settings/public.hpp"

namespace json {

    settings::SettingsManager parse_settings_config(const actuators::ActuatorProperties& actuator_properties) {
        const auto config_path = resolve_run_config_entry_path("settings_config");
        const auto config = read_json_file(config_path);
        const settings::SettingsManager settings_manager = parse_settings(config, actuator_properties);
        return settings_manager;
    }

}
