#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/actuator.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace json {

    template <typename ActuatorType>
    ActuatorType parse_actuator(const nlohmann::json& config, const std::string& key) {
        const auto& actuator_json = config.at(key);

        ActuatorType actuator{ actuators::Actuator{
            .limit_max = util::deg_to_rad(actuator_json.value("limit_max", 0.0)),
            .limit_min = util::deg_to_rad(actuator_json.value("limit_min", 0.0)),
            .tau = actuator_json.value("tau", constants::eps),
        } };

        return actuator;
    }

    actuators::ActuatorProperties parse_actuator_properties(const nlohmann::json& config) {
        actuators::Actuators actuators = {
            .elevator = parse_actuator<actuators::Elevator>(config, "elevator"),
            .aileron = parse_actuator<actuators::Aileron>(config, "aileron"),
            .rudder = parse_actuator<actuators::Rudder>(config, "rudder"),
            .flaps = parse_actuator<actuators::Flap>(config, "flaps"),
            .spoilers = parse_actuator<actuators::Spoiler>(config, "spoilers")
        };

        return { .actuators = actuators };
    }

    actuators::ActuatorProperties parse_actuator_config() {
        const auto config_path = resolve_run_config_entry_path("actuator_config");
        const auto config = read_json_file(config_path);
        return parse_actuator_properties(config);
    }

}
