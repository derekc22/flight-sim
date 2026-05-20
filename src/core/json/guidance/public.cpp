#include <nlohmann/json.hpp>
#include "core/json/guidance/private.hpp"
#include "core/json/guidance/public.hpp"
#include "core/json/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/guidance/public.hpp"

namespace json {

    guidance::GuidanceProperties parse_guidance_config(const control::ControlProperties& control_properties) {
        const auto config_path = resolve_run_config_entry_path("guidance_config");
        const auto config = read_json_file(config_path);
        return parse_guidance_properties(config, control_properties);
    }

}
