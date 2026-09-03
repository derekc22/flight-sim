#include <nlohmann/json.hpp>
#include "core/json/public/files.hpp"
#include "core/json/structural/private/parsing.hpp"
#include "core/json/structural/public/parser.hpp"
#include "simulation/structural/public/manager.hpp"

namespace json {

	structural::StructuralManager parse_structural_config()
	{
		const auto config_path = resolve_run_config_entry_path("structural_config");
		const auto config = read_json_file(config_path);
		return parse_structural_manager(config);
	}

} // namespace json
