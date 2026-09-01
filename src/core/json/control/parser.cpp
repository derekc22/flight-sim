#include <nlohmann/json.hpp>
#include "core/json/control/private/parsing.hpp"
#include "core/json/control/public/parser.hpp"
#include "core/json/public/files.hpp"
#include "simulation/control/public/manager.hpp"

namespace json {

    control::ControlManager parse_control_config(bool trim_flag) {
        const auto config_path = resolve_run_config_entry_path("control_config");
        const auto config = read_json_file(config_path);
        return parse_control_manager(config, trim_flag);
    }

}
