#include "core/json/private/detail/files.hpp"

#include "core/json/public/files.hpp"

#include <array>
#include <filesystem>
#include <format>
#include <fstream>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>

namespace json
{

	nlohmann::json read_json_file(
		const std::filesystem::path& path)
	{
		std::ifstream file(path);
		if (!file.is_open()) {
			throw std::runtime_error("failed to open file: " + path.string());
		}

		nlohmann::json config;
		file >> config;
		return config;
	}

	std::filesystem::path resolve_run_config_entry_path(
		const std::string& key)
	{
		const auto run_path = std::filesystem::path("config") / "run.json";
		const auto run_config = read_json_file(run_path);
		return resolve_config_path(run_path, run_config.at(key).get<std::string>());
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

	void dump_run_configs(
		const std::string& dir_path)
	{
		const auto run_path = std::filesystem::path("config") / "run.json";
		const auto run_config = read_json_file(run_path);
		const auto run_dir_path = std::filesystem::path(dir_path) / "run";
		std::filesystem::create_directories(run_dir_path);
		write_json(run_config, run_dir_path.string(), "run");

		for (const auto& [key, value] : run_config.items()) {
			if (!value.is_string()) {
				throw std::runtime_error(std::format("json::dump_run_configs: expected string path for key '{}'", key));
			}
			const auto config_path = resolve_config_path(run_path, value.get<std::string>());
			write_json(read_json_file(config_path), run_dir_path.string(), key);
		}
	}

	void dump_analyze_configs(
		const std::string& dir_path)
	{
		const auto analyze_path = std::filesystem::path("config") / "analyze.json";
		const auto analyze_config = read_json_file(analyze_path);
		const auto analysis_dir_path = std::filesystem::path(dir_path) / "analysis";
		std::filesystem::create_directories(analysis_dir_path);
		write_json(analyze_config, analysis_dir_path.string(), "analyze");

		const std::array<std::string, 2> groups{"linear", "nonlinear"};

		for (const auto& group : groups) {
			const auto group_dir_path = analysis_dir_path / group;
			std::filesystem::create_directories(group_dir_path);
			const auto& cfg = analyze_config.at(group);

			for (const auto& [key, value] : cfg.items()) {
				if (value.is_null()) {
					continue;
				}
				if (!value.is_string()) {
					throw std::runtime_error(
						std::format("json::dump_analyze_configs: expected string path for key '{}'", key));
				}
				const auto config_path = resolve_config_path(analyze_path, value.get<std::string>());
				write_json(read_json_file(config_path), group_dir_path.string(), key);
			}
		}
	}

} // namespace json
