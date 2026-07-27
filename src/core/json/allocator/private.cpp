#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/allocator/private.hpp"
#include "core/json/public.hpp"
#include "simulation/allocator/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/validate/public.hpp"

namespace json {

    void validate_allocator(const nlohmann::json& config) {
        if (!config.is_object()) {
            throw std::runtime_error("json::validate_allocator expected object");
        }
        if (!config.contains("Q") || !config.contains("R")) {
            throw std::runtime_error("json::validate_allocator requires Q and R");
        }
    }

    allocator::AllocatorProperties parse_allocator_properties(const nlohmann::json& config) {
        validate_allocator(config);

        Eigen::MatrixXd Q = parse_MatrixXd(config.at("Q"));
        Eigen::MatrixXd R = parse_MatrixXd(config.at("R"));

        const std::string context = "json::parse_allocator_properties";
        util::validate_shape(Q, constants::virtual_input_dim, constants::virtual_input_dim, context, "Q");
        util::validate_shape(R, constants::input_dim, constants::input_dim, context, "R");

        return {
            .Q = Q,
            .R = R
        };
    }
}
