#include <nlohmann/json.hpp>
#include "core/json/aerodynamics/private/parsing.hpp"
#include "core/json/aerodynamics/public/parser.hpp"
#include "core/json/public/files.hpp"
#include "simulation/aerodynamics/public/manager.hpp"

namespace json {

    aerodynamics::AerodynamicsManager parse_aerodynamics_config() {
        const auto config_path = resolve_run_config_entry_path("aerodynamics_config");
        const auto config = read_json_file(config_path);
        return parse_aerodynamics_manager(config);
    }

}
