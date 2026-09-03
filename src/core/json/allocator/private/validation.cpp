#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/allocator/private/validation.hpp"

namespace json {

	void validate_allocator(
	    const nlohmann::json& config)
	{
		if (!config.is_object()) {
			throw std::runtime_error("json::validate_allocator expected object");
		}
		if (!config.contains("Q") || !config.contains("R")) {
			throw std::runtime_error("json::validate_allocator requires Q and R");
		}
	}

} // namespace json
