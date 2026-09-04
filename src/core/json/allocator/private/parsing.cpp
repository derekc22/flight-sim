#include "core/json/allocator/private/parsing.hpp"

#include "core/json/allocator/private/validation.hpp"
#include "core/json/public/data/helpers.hpp"
#include "simulation/allocator/public/manager.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/util/public/validation.hpp"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>

namespace json
{

	allocator::AllocatorManager parse_allocator_manager(
		const nlohmann::json& config)
	{
		validate_allocator(config);

		Eigen::MatrixXd Q = parse_MatrixXd(config.at("Q"));
		Eigen::MatrixXd R = parse_MatrixXd(config.at("R"));

		const std::string context = "json::parse_allocator_manager";
		util::validate_shape(Q, constants::virtual_input_dim, constants::virtual_input_dim, context, "Q");
		util::validate_shape(R, constants::input_dim, constants::input_dim, context, "R");

		return {.Q = Q, .R = R};
	}
} // namespace json
