#pragma once
#include <nlohmann/json.hpp>

namespace json {

	void validate_NEDFrameECEF_initialization(const nlohmann::json& frame_json);
	void validate_FRDFrameECEF_initialization(const nlohmann::json& frame_json);
	void validate_FRDFrameNED_initialization(const nlohmann::json& frame_json);

	void validate_initialization_config(const nlohmann::json& config, bool trim_flag);

} // namespace json
