#pragma once
#include <nlohmann/json.hpp>
#include "simulation/avionics/public.hpp"

namespace json {

    void validate_attitude_heading_reference_system_json(const nlohmann::json& ahrs_json);
    avionics::AttitudeHeadingReferenceSystem parse_attitude_heading_reference_system(const nlohmann::json& ahrs_json);
    avionics::AvionicsProperties parse_avionics_properties(const nlohmann::json& config);
}
