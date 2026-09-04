#include "core/json/private/detail/files.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>

namespace json
{

	std::filesystem::path resolve_config_path(
		const std::filesystem::path& entry_path,
		const std::string& config_path)
	{
		const std::filesystem::path path{config_path};
		if (path.is_absolute()) {
			return path;
		}
		return entry_path.parent_path() / path;
	}

	void write_json(
		const nlohmann::json& config,
		const std::string& dir_path,
		const std::string& fname)
	{
		const auto file_path = std::filesystem::path(dir_path) / (fname + ".json");
		std::ofstream file_json(file_path);
		if (!file_json.is_open()) {
			throw std::runtime_error("Failed to open file: " + file_path.string());
		}

		file_json << config.dump(4) << "\n";
		file_json.close();

		std::string log_str = "File saved successfully to " + file_path.string();
		spdlog::info(log_str);
	}

} // namespace json
