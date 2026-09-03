#include <nlohmann/json.hpp>
#include "core/json/guidance/private/parsing.hpp"
#include "core/json/guidance/public/parser.hpp"
#include "core/json/public/files.hpp"
#include "simulation/control/public/manager.hpp"
#include "simulation/guidance/public/manager.hpp"

namespace json {

	guidance::GuidanceManager parse_guidance_config(
	    const control::ControlManager& control_manager)
	{
		const auto config_path = resolve_run_config_entry_path("guidance_config");
		const auto config = read_json_file(config_path);
		return parse_guidance_manager(config, control_manager);
	}

} // namespace json
