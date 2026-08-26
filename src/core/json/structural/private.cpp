#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "core/json/structural/private.hpp"
#include "core/json/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    structural::StructuralProperties parse_structural_properties(const nlohmann::json& config) {
        const auto& geometries_json = config.at("geometries");
        if (!geometries_json.is_array()) { 
            throw std::runtime_error("json::parse_structural_config expected 'geometries' to be an array"); 
        }

        std::vector<structural::Geometry> geometries;
        geometries.reserve(geometries_json.size());
        for (const auto& geom_json : geometries_json) {
            geometries.push_back(structural::Geometry{
                .id = geom_json.at("id").get<std::string>(),
                .mass = geom_json.at("mass").get<double>(),
                .x_size = geom_json.at("x_size").get<double>(),
                .y_size = geom_json.at("y_size").get<double>(),
                .z_size = geom_json.at("z_size").get<double>(),
                .pB_geomB = parse_Vector3d(geom_json.at("pB_geomB")),
            });
        }

        return { geometries };
    }

}
