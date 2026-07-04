#include <nlohmann/json.hpp>
#include "simulation/runner/public.hpp"

namespace json {

    void validate_simulation_hz_json(const nlohmann::json& hz_config, const nlohmann::json& enable_config);
    void validate_enable_json(const nlohmann::json& enable_config);
    runner::JSONOptions parse_runner_options(const nlohmann::json& config);

}
