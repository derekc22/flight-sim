#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	std::filesystem::path resolve_config_path(const std::filesystem::path& entry_path, const std::string& config_path);

	void write_json(const nlohmann::json& config, const std::string& dir_path, const std::string& fname);
} // namespace json
