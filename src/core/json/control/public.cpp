#include <nlohmann/json.hpp>
#include "core/json/control/private.hpp"
#include "core/json/control/public.hpp"
#include "core/json/public.hpp"
#include "simulation/control/public.hpp"

namespace json {

    control::ControlProperties parse_control_config(bool trim_flag) {
        const auto config_path = resolve_run_config_entry_path("control_config");
        const auto config = read_json_file(config_path);
        return parse_control_properties(config, trim_flag);
    }

}
