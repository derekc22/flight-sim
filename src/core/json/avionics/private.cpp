#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/avionics/private.hpp"
#include "simulation/avionics/public.hpp"

namespace json {

    void validate_attitude_heading_reference_system_json(const nlohmann::json& ahrs_json) {
        if (!ahrs_json.is_object()) {
            throw std::runtime_error("json::validate_attitude_heading_reference_system_json expected attitude_heading_reference_system object");
        }
        if (!ahrs_json.contains("Kp")) {
            throw std::runtime_error("json::validate_attitude_heading_reference_system_json: Kp not present");
        }
        if (!ahrs_json.contains("Ki")) {
            throw std::runtime_error("json::validate_attitude_heading_reference_system_json: Ki not present");
        }
        if (!ahrs_json.contains("fB_tol_scale")) {
            throw std::runtime_error("json::validate_attitude_heading_reference_system_json: fB_tol_scale not present");
        }
    }

    avionics::AttitudeHeadingReferenceSystem parse_attitude_heading_reference_system(const nlohmann::json& ahrs_json) {
        validate_attitude_heading_reference_system_json(ahrs_json);

        return {
            .Kp = ahrs_json.at("Kp").get<double>(),
            .Ki = ahrs_json.at("Ki").get<double>(),
            .fB_tol_scale = ahrs_json.at("fB_tol_scale").get<double>()
        };
    }

    avionics::AvionicsProperties parse_avionics_properties(const nlohmann::json& config) {
        const auto& ahrs_json = config.at("attitude_heading_reference_system");

        avionics::Avionics avionics = {
            .AHRS = parse_attitude_heading_reference_system(ahrs_json)
        };

        return { .avionics = avionics };
    }

}
