#include <nlohmann/json.hpp>
#include "core/json/public.hpp"
#include "core/json/structural/private.hpp"
#include "core/json/structural/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    structural::StructuralProperties parse_structural_config() {
        const auto config_path = resolve_run_config_entry_path("structural_config");
        const auto config = read_json_file(config_path);
        return parse_structural_properties(config);
    }

}
