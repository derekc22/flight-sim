#pragma once
#include <filesystem>
#include <string>

namespace json
{

	std::filesystem::path resolve_config_path(const std::filesystem::path& entry_path, const std::string& config_path);
} // namespace json
