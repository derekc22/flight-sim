#include "core/json/private/detail/files.hpp"

#include "core/json/public/files.hpp"

#include <array>
#include <filesystem>
#include <format>
#include <fstream>
#include <nlohmann/json.hpp>
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

	void dump_run_configs(
		const std::string& dir_path)
	{
		const auto run_path = std::filesystem::path("config") / "run.json";
		const auto run_config = read_json_file(run_path);
		write_json(run_config, dir_path, "run");

		for (const auto& [key, value] : run_config.items()) {
			if (!value.is_string()) {
				throw std::runtime_error(std::format("json::dump_run_configs: expected string path for key '{}'", key));
			}
			const auto config_path = resolve_config_path(run_path, value.get<std::string>());
			write_json(read_json_file(config_path), dir_path, key);
		}
	}

	void dump_analyze_configs(
		const std::string& dir_path)
	{
		const auto analyze_path = std::filesystem::path("config") / "analyze.json";
		const auto analyze_config = read_json_file(analyze_path);
		write_json(analyze_config, dir_path, "analyze");

		const std::array<nlohmann::json, 2> cfgs{analyze_config.at("linear"), analyze_config.at("nonlinear")};

		for (const auto& cfg : cfgs) {
			for (const auto& [key, value] : cfg.items()) {
				if (value.is_null()) {
					continue;
				}
				if (!value.is_string()) {
					throw std::runtime_error(
						std::format("json::dump_analyze_configs: expected string path for key '{}'", key));
				}
				const auto config_path = resolve_config_path(analyze_path, value.get<std::string>());
				write_json(read_json_file(config_path), dir_path, key);
			}
		}
	}

} // namespace json
