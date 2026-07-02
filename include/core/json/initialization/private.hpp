#pragma once
#include <optional>
#include <nlohmann/json.hpp>
#include "core/json/initialization/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/frames/public.hpp"

namespace json {

    struct ParsedStepOptions : frames::StandardFrameFieldsOptional {
        std::optional<geography::Latitude> lat;
        std::optional<geography::Longitude> lon;
        std::optional<geography::GeometricAltitude> alt;
        std::optional<aerodynamics::AngleOfAttack> alpha;
        std::optional<aerodynamics::SideslipAngle> beta;
    };

    ParsedStepOptions parse_step_options(const nlohmann::json& frame_json);
    vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(const nlohmann::json& frame_json);
    vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(const nlohmann::json& frame_json);
    vehicles::FRDFrameNEDStepOptions parse_FRDFrameNED_step_options(const nlohmann::json& frame_json);
    void validate_NEDFrameECEF_initialization(const nlohmann::json& frame_json);
    void validate_FRDFrameECEF_initialization(const nlohmann::json& frame_json);
    void validate_FRDFrameNED_initialization(const nlohmann::json& frame_json);
    void validate_initialization_config(const nlohmann::json& config, bool trim_flag);
}
