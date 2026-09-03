#pragma once
#include <filesystem>
#include <string>
#include <nlohmann/json.hpp>

namespace json {

	nlohmann::json read_json_file(const std::filesystem::path& path);
	std::filesystem::path resolve_run_config_entry_path(const std::string& key);

	void dump_run_configs(const std::string& dir_path);
	void dump_analyze_configs(const std::string& dir_path);

} // namespace json
