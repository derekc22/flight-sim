#include "core/json/estimation/public/parser.hpp"

#include "core/json/estimation/private/parsing.hpp"
#include "core/json/public/files.hpp"
#include "simulation/estimation/public/manager.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	estimation::EstimationManager parse_estimation_config(
		bool trim_flag)
	{
		const auto config_path = resolve_run_config_entry_path("estimation_config");
		const auto config = read_json_file(config_path);
		return parse_estimation_manager(config, trim_flag);
	}
} // namespace json
