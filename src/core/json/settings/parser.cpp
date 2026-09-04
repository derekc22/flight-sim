#include "core/json/settings/public/parser.hpp"

#include "core/json/public/files.hpp"
#include "core/json/settings/private/parsing.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/settings/public/data/types.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	settings::Settings parse_settings_config(
		const actuators::ActuatorManager& actuator_manager)
	{
		const auto config_path = resolve_run_config_entry_path("settings_config");
		const auto config = read_json_file(config_path);
		const settings::Settings simulation_settings = parse_settings(config, actuator_manager);
		return simulation_settings;
	}

} // namespace json
