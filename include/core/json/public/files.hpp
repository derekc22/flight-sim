#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	nlohmann::json read_json_file(const std::filesystem::path& path);
	std::filesystem::path resolve_run_config_entry_path(const std::string& key);
	void write_json(const nlohmann::json& config, const std::string& dir_path, const std::string& fname);

	void dump_run_configs(const std::string& dir_path);
	void dump_analyze_configs(const std::string& dir_path);

} // namespace json
