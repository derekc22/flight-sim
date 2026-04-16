#include <string>
#include <nlohmann/json.hpp>
#include "core/json/avionics.hpp"
#include "core/json/json.hpp"
#include "simulation/constants/constants.hpp"

namespace json {

    double parse_sensor_scalar_bias(const nlohmann::json& sensor_json) {
        return sensor_json.at("bias").get<double>();
    }

    Eigen::Vector3d parse_sensor_3d_bias(const nlohmann::json& sensor_json) {
        return parse_Vector3d(sensor_json.at("bias"));
    }

    template <typename SensorType>
    SensorType parse_sensor(const nlohmann::json& config, const std::string& key) {
        const auto& sensor_json = config.at(key);
        const bool has_vector_bias = sensor_json.at("bias").is_array();
        const double bias = has_vector_bias ? 0.0 : parse_sensor_scalar_bias(sensor_json);
        const Eigen::Vector3d bias_3d = has_vector_bias ? parse_sensor_3d_bias(sensor_json) : constants::Zero3;

        SensorType sensor{ avionics::Sensor(
            sensor_json.value("mean", 0.0),
            sensor_json.value("stddev", 0.0),
            bias,
            bias_3d,
            sensor_json.value("tau", constants::eps)
        ) };

        return sensor;
    }

    avionics::AvionicsProperties parse_avionics_properties(const nlohmann::json& config) {
        avionics::AvionicsSensors sensors = {
            .aoa_vane = parse_sensor<avionics::AngleOfAttackVane>(config, "angle_of_attack_vane"),
            .accelerometer = parse_sensor<avionics::Accelerometer>(config, "accelerometer"),
            .gyro = parse_sensor<avionics::Gyroscope>(config, "gyroscope"),
            .pitot_tube = parse_sensor<avionics::PitotTube>(config, "pitot_tube"),
            .static_port = parse_sensor<avionics::StaticPort>(config, "static_port"),
            .tat_probe = parse_sensor<avionics::TotalAirTemperatureProbe>(config, "total_air_temperature_probe"),
            .gnss = parse_sensor<avionics::GNSSReceiver>(config, "gnss_receiver"),
            .magnetometer = parse_sensor<avionics::Magnetometer>(config, "magnetometer"),
        };

        return { .sensors = sensors };
    }


    avionics::AvionicsProperties parse_avionics_config() {
        const auto config_path = resolve_run_config_entry_path("avionics_config");
        const auto config = read_json_file(config_path);
        return parse_avionics_properties(config);
    }

}
