#pragma once
#include <nlohmann/json.hpp>
#include "core/json/initialization/private/data/types.hpp"
#include "simulation/vehicles/public/data/types.hpp"

namespace json {

	ParsedStepOptions parse_step_options(const nlohmann::json& frame_json);

	vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(const nlohmann::json& frame_json);
	vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(const nlohmann::json& frame_json);
	vehicles::FRDFrameNEDStepOptions parse_FRDFrameNED_step_options(const nlohmann::json& frame_json);

} // namespace json
