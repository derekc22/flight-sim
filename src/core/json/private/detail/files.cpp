#include "core/json/private/detail/files.hpp"

#include <filesystem>
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

} // namespace json
