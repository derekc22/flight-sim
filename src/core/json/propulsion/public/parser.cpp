#include "core/json/propulsion/public/parser.hpp"

#include "core/json/propulsion/private/parsing.hpp"
#include "core/json/public/files.hpp"
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/manager.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	propulsion::PropulsionManager parse_propulsion_config(
		structural::StructuralManager& structural_manager)
	{
		const auto config_path = resolve_run_config_entry_path("propulsion_config");
		const auto config = read_json_file(config_path);
		return parse_propulsion_manager(config, structural_manager);
	}

} // namespace json
