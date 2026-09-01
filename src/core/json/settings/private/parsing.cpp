#include <nlohmann/json.hpp>
#include "core/json/settings/private/parsing.hpp"
#include "core/json/settings/private/validation.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/settings/public/data/types.hpp"

namespace json {

    settings::Settings parse_settings(const nlohmann::json& config, const actuators::ActuatorManager& actuator_manager) {
        settings::Settings simulation_settings;

        validate_actuator_settings_json(config);
        const auto& actuator_settings_json = config.at("actuators");
        const auto& fixed_actuator_inputs_json = actuator_settings_json.at("fixed_actuator_inputs");
        if (fixed_actuator_inputs_json.contains("flap")) {
            simulation_settings.actuator_settings.fixed_actuator_inputs.flap =
                fixed_actuator_inputs_json.at("flap").get<double>();
        }
        if (fixed_actuator_inputs_json.contains("spoiler")) {
            simulation_settings.actuator_settings.fixed_actuator_inputs.spoiler =
                fixed_actuator_inputs_json.at("spoiler").get<double>();
        }
        validate_actuator_settings(simulation_settings.actuator_settings, actuator_manager);


        validate_avionics_settings_json(config);
        const auto& avionics_settings_json = config.at("avionics");
        simulation_settings.avionics_settings.use_ins = avionics_settings_json.at("use_ins").get<bool>();

        return simulation_settings;
    }

}
