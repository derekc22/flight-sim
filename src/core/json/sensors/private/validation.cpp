#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/sensors/private/validation.hpp"

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

}
