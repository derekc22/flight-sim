#include <nlohmann/json.hpp>
#include "core/json/sensors/private/parsing.hpp"
#include "simulation/sensors/public/manager.hpp"

namespace json {

    sensors::SensorManager parse_sensor_manager(const nlohmann::json& config) {
        sensors::Sensors sensors = {
            .aoa_vane = parse_sensor<sensors::AngleOfAttackVane>(config, "angle_of_attack_vane"),
            .accelerometer = parse_sensor<sensors::Accelerometer>(config, "accelerometer"),
            .gyro = parse_sensor<sensors::Gyroscope>(config, "gyroscope"),
            .pitot_tube = parse_sensor<sensors::PitotTube>(config, "pitot_tube"),
            .static_port = parse_sensor<sensors::StaticPort>(config, "static_port"),
            .tat_probe = parse_sensor<sensors::TotalAirTemperatureProbe>(config, "total_air_temperature_probe"),
            .gnss = parse_sensor<sensors::GNSSReceiver>(config, "gnss_receiver"),
            .magnetometer = parse_sensor<sensors::Magnetometer>(config, "magnetometer"),
        };

        return { .sensors = sensors };
    }

}
