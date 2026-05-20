#pragma once
#include <optional>
#include <nlohmann/json.hpp>
#include "core/json/initialization/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/vehicles/public.hpp"

namespace json {

    struct ParsedStepOptions {
        std::optional<dynamics::HomogeneousFrameTransformationMatrix> H;
        std::optional<dynamics::OrientationMatrix> C;
        std::optional<dynamics::Position> p;
        std::optional<dynamics::OrientationQuaternion> q;
        std::optional<dynamics::EulerAngles> eul;
        std::optional<dynamics::OrientationMatrixRate> C_dot;
        std::optional<dynamics::OrientationQuaternionRate> q_dot;
        std::optional<dynamics::AngularVelocity> w;
        std::optional<dynamics::EulerAngleRates> eul_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq;
        std::optional<dynamics::TranslationalVelocity> v;
        std::optional<geography::Latitude> lat;
        std::optional<geography::Longitude> lon;
        std::optional<geography::Altitude> alt;
        std::optional<aerodynamics::AngleOfAttack> alpha;
        std::optional<aerodynamics::SideslipAngle> beta;
    };

    ParsedStepOptions parse_step_options(const nlohmann::json& frame_json);
    vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(const nlohmann::json& frame_json);
    vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(const nlohmann::json& frame_json);
    vehicles::FRDFrameNEDStepOptions parse_FRDFrameNED_step_options(const nlohmann::json& frame_json);
    void _validate_NEDFrameECEF_initialization_config(const nlohmann::json& frame_json);
    void _validate_FRDFrameECEF_initialization_config(const nlohmann::json& frame_json);
    void _validate_FRDFrameNED_initialization_config(const nlohmann::json& frame_json);
    void _validate_initialization_config(const nlohmann::json& config, bool trim_bool);
}
