#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/actuator.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace json {

    void validate_actuator_json(const nlohmann::json& actuator_json) {
        if (!actuator_json.contains("limit_max")) { throw std::runtime_error("json::validate_actuator_json: actuator maximum limit not present"); }
        if (!actuator_json.contains("limit_min")) { throw std::runtime_error("json::validate_actuator_json: actuator minimum limit not present"); }
        if (!actuator_json.contains("tau")) { throw std::runtime_error("json::validate_actuator_json: actuator tau not present"); }

        const double limit_max = actuator_json.at("limit_max").get<double>();
        const double limit_min = actuator_json.at("limit_min").get<double>();
        const double tau = actuator_json.at("tau").get<double>();

        if (limit_max < limit_min) { throw std::runtime_error("json::validate_actuator_json: actuator maximum limit must be greater than or equal to minimum limit"); }
        if (tau < 0.0) { throw std::runtime_error("json::validate_actuator_json: actuator tau must be non-negative"); }
    }

    template <typename ActuatorType>
    ActuatorType parse_actuator(const nlohmann::json& config, const std::string& key) {
        const auto& actuator_json = config.at(key);
        validate_actuator_json(actuator_json);

        ActuatorType actuator{ actuators::Actuator{
            .limit_max = util::deg_to_rad(actuator_json.at("limit_max").get<double>()),
            .limit_min = util::deg_to_rad(actuator_json.at("limit_min").get<double>()),
            .tau = actuator_json.at("tau").get<double>(),
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
