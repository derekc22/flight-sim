#include "core/json/structural/private/parsing.hpp"

#include "core/json/public/data/helpers.hpp"
#include "simulation/structural/public/manager.hpp"
#include "simulation/util/public/validation.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace json
{

	structural::StructuralManager parse_structural_manager(
		const nlohmann::json& config)
	{
		const auto& geometries_json = config.at("geometries");
		if (!geometries_json.is_array()) {
			throw std::runtime_error("json::parse_structural_config expected 'geometries' to be an array");
		}

		std::vector<structural::Geometry> geometries;
		geometries.reserve(geometries_json.size());
		for (const auto& geom_json : geometries_json) {
			Eigen::Matrix3d CBL = parse_Matrix3d(geom_json.at("CBL"));
			util::validate_orthonormal(CBL, "json::parse_structural_manager", "geometry CBL");
			geometries.push_back(structural::Geometry{
				.id = geom_json.at("id").get<std::string>(),
				.mass = geom_json.at("mass").get<double>(),
				.x_size = geom_json.at("x_size").get<double>(),
				.y_size = geom_json.at("y_size").get<double>(),
				.z_size = geom_json.at("z_size").get<double>(),
				.pB_geomB = parse_Vector3d(geom_json.at("pB_geomB")),
				.CBL = CBL
			});
		}

		return {geometries};
	}

} // namespace json
