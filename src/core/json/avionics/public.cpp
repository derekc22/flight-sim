#include <nlohmann/json.hpp>
#include "core/json/avionics/private.hpp"
#include "core/json/avionics/public.hpp"
#include "core/json/public.hpp"
#include "simulation/avionics/public.hpp"

namespace json {

    avionics::AvionicsProperties parse_avionics_config() {
        const auto config_path = resolve_run_config_entry_path("avionics_config");
        const auto config = read_json_file(config_path);
        return parse_avionics_properties(config);
    }

}
