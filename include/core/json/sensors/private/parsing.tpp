#pragma once
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/public/data/helpers.hpp"
#include "core/json/sensors/private/validation.hpp"
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/constants/public.hpp"

namespace json {

    template <typename SensorType>
    SensorType parse_sensor(const nlohmann::json& config, const std::string& key) {
        const auto& sensor_json = config.at(key);
        validate_sensor_json(sensor_json);

        bool has_vector_bias = sensor_json.at("bias").is_array();
        double bias = has_vector_bias ? 0.0 : sensor_json.at("bias").get<double>();
        Eigen::Vector3d bias_3d = has_vector_bias ? parse_Vector3d(sensor_json.at("bias")) : constants::Zero3;

        SensorType sensor{ sensors::Sensor(
            sensor_json.at("mean").get<double>(),
            sensor_json.at("stddev").get<double>(),
            bias,
            bias_3d,
            sensor_json.at("tau").get<double>()
        ) };

        return sensor;
    }
}
