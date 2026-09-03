#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/control/private/validation.hpp"

namespace json {

	void validate_controllers(
	    const nlohmann::json& controllers_json,
	    bool trim_flag)
	{
		bool has_attitude = controllers_json.contains("attitude");
		bool has_velocity = controllers_json.contains("velocity");
		bool has_linear_quadratic = controllers_json.contains("linear_quadratic");
		bool has_nonlinear = controllers_json.contains("nonlinear");

		if (has_linear_quadratic && !trim_flag) {
			throw std::runtime_error("json::validate_controllers: linear_quadratic control requires trim");
		}

		if (has_attitude && has_linear_quadratic) {
			throw std::runtime_error(
			    "json::validate_controllers: attitude and linear_quadratic control laws cannot both be present");
		}
		if (has_attitude && has_nonlinear) {
			throw std::runtime_error(
			    "json::validate_controllers: attitude and nonlinear control laws cannot both be present");
		}

		if (has_linear_quadratic && has_nonlinear) {
			throw std::runtime_error(
			    "json::validate_controllers: linear_quadratic and nonlinear control laws cannot both be present");
		}

		if (has_velocity && has_linear_quadratic) {
			throw std::runtime_error(
			    "json::validate_controllers: velocity and linear_quadratic control laws cannot both be present");
		}
		if (has_velocity && has_nonlinear) {
			throw std::runtime_error(
			    "json::validate_controllers: velocity and nonlinear control laws cannot both be present");
		}
	}

} // namespace json
