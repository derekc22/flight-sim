#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/avionics/private.hpp"
#include "simulation/avionics/public.hpp"

namespace json {

    void validate_sensor_json(const nlohmann::json& sensor_json) {
        if (!sensor_json.contains("mean")) { 
            throw std::runtime_error("json::validate_sensor_json: sensor noise mean not present"); 
        }
        if (!sensor_json.contains("stddev")) { 
            throw std::runtime_error("json::validate_sensor_json: sensor noise stddev not present"); 
        }
        if (!sensor_json.contains("bias")) { 
            throw std::runtime_error("json::validate_sensor_json: sensor bias not present"); 
        }
        if (!sensor_json.contains("tau")) { 
            throw std::runtime_error("json::validate_sensor_json: sensor tau not present"); 
        }

        double stddev = sensor_json.at("stddev").get<double>();
        double tau = sensor_json.at("tau").get<double>();

        if (stddev < 0.0) { 
            throw std::runtime_error("json::validate_sensor_json: sensor noise stddev must be non-negative"); 
        }
        if (tau < 0.0) { 
            throw std::runtime_error("json::validate_sensor_json: sensor tau must be non-negative"); 
        }
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

}
