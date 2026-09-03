#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "core/json/actuators/private/parsing.hpp"
#include "core/json/actuators/private/validation.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/structural/public/manager.hpp"

namespace json {

    actuators::PropellerAssembly parse_propellers(const nlohmann::json& propellers_json, structural::StructuralManager& structural_manager, const Eigen::Vector3d& n_prop) {
        validate_propellers_json(propellers_json);

        std::vector<std::string> geometry_ids = propellers_json.at("geometry_ids").get<std::vector<std::string>>();
        double y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();
        double spin_inertia = 0.0;

        for (const std::string& geometry_id : geometry_ids) {
            const structural::Geometry& geom = structural_manager.get_geometry(geometry_id);
            y_min = std::min(y_min, geom.pB_geomB(1) - 0.5 * geom.y_size);
            y_max = std::max(y_max, geom.pB_geomB(1) + 0.5 * geom.y_size);
            spin_inertia += structural_manager.compute_spin_inertia(geom, n_prop);
        }

        const double diameter = y_max - y_min;
        if (diameter < constants::eps) { 
            throw std::runtime_error("json::parse_propellers: computed propeller diameter must be positive"); 
        }
        if (spin_inertia < constants::eps) { 
            throw std::runtime_error("json::parse_propellers: computed propeller spin inertia must be positive"); 
        }

        return {
            .geometry_ids = geometry_ids,
            .spin_sign = propellers_json.at("spin_sign").get<double>(),
            .thrust_coeff = propellers_json.at("thrust_coeff").get<double>(),
            .torque_coeff = propellers_json.at("torque_coeff").get<double>(),
            .diameter = diameter,
            .spin_inertia = spin_inertia
        };
    }

    actuators::ActuatorManager parse_actuator_manager(const nlohmann::json& config, structural::StructuralManager& structural_manager) {
        const auto& surface_actuator_json = config.at("surfaces");
        const auto& propulsor_actuator_json = config.at("propulsors");

        actuators::SurfaceActuators surface_actuators = {
            .elevator = parse_surface_actuator<actuators::ElevatorActuator>(surface_actuator_json, "elevator"),
            .aileron = parse_surface_actuator<actuators::AileronActuator>(surface_actuator_json, "aileron"),
            .rudder = parse_surface_actuator<actuators::RudderActuator>(surface_actuator_json, "rudder"),
            .flap = parse_surface_actuator<actuators::FlapActuator>(surface_actuator_json, "flap"),
            .spoiler = parse_surface_actuator<actuators::SpoilerActuator>(surface_actuator_json, "spoiler")
        };

        actuators::PropulsorActuators propulsor_actuators = {
            .front_propulsor = parse_propulsor_actuator<actuators::FrontPropulsorActuator>(propulsor_actuator_json, "front_propulsor", structural_manager),
            .left_propulsor = parse_propulsor_actuator<actuators::LeftPropulsorActuator>(propulsor_actuator_json, "left_propulsor", structural_manager),
            .right_propulsor = parse_propulsor_actuator<actuators::RightPropulsorActuator>(propulsor_actuator_json, "right_propulsor", structural_manager),
        };

        return { .surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators };
    }

}
