#include <nlohmann/json.hpp>
#include "core/json/runner/private.hpp"
#include "core/json/runner/public.hpp"
#include "core/json/public.hpp"
#include "simulation/runner/public.hpp"

namespace json {

    runner::JSONOptions parse_runner_config() {
        const auto config_path = resolve_run_config_entry_path("runner_config");
        const auto config = read_json_file(config_path);
        return parse_runner_options(config);
    }

}
