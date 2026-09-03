#include <nlohmann/json.hpp>
#include "core/json/avionics/private/parsing.hpp"
#include "core/json/avionics/public/parser.hpp"
#include "core/json/public/files.hpp"
#include "simulation/avionics/public/manager.hpp"

namespace json {

	avionics::AvionicsManager parse_avionics_config()
	{
		const auto config_path = resolve_run_config_entry_path("avionics_config");
		const auto config = read_json_file(config_path);
		return parse_avionics_manager(config);
	}

} // namespace json
