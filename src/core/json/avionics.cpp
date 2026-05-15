#include <string>
#include <nlohmann/json.hpp>
#include "core/json/avionics.hpp"
#include "core/json/json.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/constants/public.hpp"

namespace json {

    void validate_sensor_json(const nlohmann::json& sensor_json) {
        if (!sensor_json.contains("mean")) { throw std::runtime_error("json::validate_sensor_json: sensor noise mean not present"); }
        if (!sensor_json.contains("stddev")) { throw std::runtime_error("json::validate_sensor_json: sensor noise stddev not present"); }
        if (!sensor_json.contains("bias")) { throw std::runtime_error("json::validate_sensor_json: sensor bias not present"); }
        if (!sensor_json.contains("tau")) { throw std::runtime_error("json::validate_sensor_json: sensor tau not present"); }

        double stddev = sensor_json.at("stddev").get<double>();
        double tau = sensor_json.at("tau").get<double>();

        if (stddev < 0.0) { throw std::runtime_error("json::validate_sensor_json: sensor noise stddev must be non-negative"); }
        if (tau < 0.0) { throw std::runtime_error("json::validate_sensor_json: sensor tau must be non-negative"); }
    }

    template <typename SensorType>
    SensorType parse_sensor(const nlohmann::json& config, const std::string& key) {
        const auto& sensor_json = config.at(key);
        validate_sensor_json(sensor_json);

        bool has_vector_bias = sensor_json.at("bias").is_array();
        double bias = has_vector_bias ? 0.0 : sensor_json.at("bias").get<double>();
        Eigen::Vector3d bias_3d = has_vector_bias ? parse_Vector3d(sensor_json.at("bias")) : constants::Zero3;

        SensorType sensor{ avionics::Sensor(
            sensor_json.at("mean").get<double>(),
            sensor_json.at("stddev").get<double>(),
            bias,
            bias_3d,
            sensor_json.at("tau").get<double>()
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
