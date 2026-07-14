#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "core/json/actuators/private.hpp"
#include "core/json/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    void validate_actuator_json(const nlohmann::json& actuator_json) {
        if (!actuator_json.contains("limit_max")) { 
            throw std::runtime_error("json::validate_actuator_json: actuator maximum limit not present"); 
        }
        if (!actuator_json.contains("limit_min")) { 
            throw std::runtime_error("json::validate_actuator_json: actuator minimum limit not present"); 
        }
        if (!actuator_json.contains("tau")) { 
            throw std::runtime_error("json::validate_actuator_json: actuator tau not present"); 
        }

        double limit_max = actuator_json.at("limit_max").get<double>();
        double limit_min = actuator_json.at("limit_min").get<double>();
        double tau = actuator_json.at("tau").get<double>();

        if (limit_max < limit_min) { 
            throw std::runtime_error("json::validate_actuator_json: actuator maximum limit must be greater than or equal to minimum limit"); 
        }
        if (tau < 0.0) { 
            throw std::runtime_error("json::validate_actuator_json: actuator tau must be non-negative"); 
        }
    }

    void validate_propulsor_actuator_json(const nlohmann::json& propulsor_actuator_json) {
        validate_actuator_json(propulsor_actuator_json);
        if (!propulsor_actuator_json.contains("geometry_id")) { 
            throw std::runtime_error("json::validate_propulsor_actuator_json: propulsor geometry_id not present"); 
        }
        if (!propulsor_actuator_json.contains("inclination_angle")) { 
            throw std::runtime_error("json::validate_propulsor_actuator_json: propulsor inclination_angle not present"); 
        }
        if (!propulsor_actuator_json.contains("toe_angle")) { 
            throw std::runtime_error("json::validate_propulsor_actuator_json: propulsor toe_angle not present"); 
        }
        
        double limit_min = propulsor_actuator_json.at("limit_min").get<double>();
        if (limit_min < 0.0) { 
            throw std::runtime_error("json::validate_propulsor_actuator_json: propulsor limit_min must be non-negative"); 
        }
    }

    void validate_propulsor_actuator_placement(const Eigen::Vector3d& p_propulsor, const std::string& key) {
        if (key == "front_propulsor") {
            if (p_propulsor(1) != 0.0) { 
                throw std::runtime_error("json::validate_propulsor_actuator_placement: front propulsor must have p_propulsor[1] = 0"); 
            }
        }
        if (key == "left_propulsor") {
            if (p_propulsor(1) > 0.0) { 
                throw std::runtime_error("json::validate_propulsor_actuator_placement: left propulsor must have p_propulsor[1] <= 0"); 
            }
        }
        if (key == "right_propulsor") {
            if (p_propulsor(1) < 0.0) { 
                throw std::runtime_error("json::validate_propulsor_actuator_placement: right propulsor must have p_propulsor[1] >= 0"); 
            }
        }
    }

    void validate_propellers_json(const nlohmann::json& propellers_json) {
        if (!propellers_json.contains("geometry_ids")) { 
            throw std::runtime_error("json::validate_propellers_json: propellers geometry_ids not present"); 
        }
        if (!propellers_json.at("geometry_ids").is_array()) { 
            throw std::runtime_error("json::validate_propellers_json: propellers geometry_ids must be an array"); 
        }
        if (propellers_json.at("geometry_ids").empty()) { 
            throw std::runtime_error("json::validate_propellers_json: propellers geometry_ids cannot be empty"); 
        }
        if (!propellers_json.contains("spin_sign")) { 
            throw std::runtime_error("json::validate_propellers_json: propellers spin_sign not present"); 
        }
        if (!propellers_json.contains("thrust_coeff")) { 
            throw std::runtime_error("json::validate_propellers_json: propellers thrust_coeff not present"); 
        }
        if (!propellers_json.contains("torque_coeff")) { 
            throw std::runtime_error("json::validate_propellers_json: propellers torque_coeff not present"); 
        }

        double spin_sign = propellers_json.at("spin_sign").get<double>();
        double thrust_coeff = propellers_json.at("thrust_coeff").get<double>();
        double torque_coeff = propellers_json.at("torque_coeff").get<double>();

        if (std::abs(std::abs(spin_sign) - 1.0) > constants::eps) { 
            throw std::runtime_error("json::validate_propellers_json: propellers spin_sign must be +1 or -1"); 
        }
        if (thrust_coeff <= 0.0) { 
            throw std::runtime_error("json::validate_propellers_json: propellers thrust_coeff must be positive"); 
        }
        if (torque_coeff < 0.0) { 
            throw std::runtime_error("json::validate_propellers_json: propellers torque_coeff must be non-negative"); 
        }
    }

    actuators::PropellerAssembly parse_propellers(const nlohmann::json& propellers_json, structural::StructuralProperties& structural_properties, const Eigen::Vector3d& n_prop) {
        validate_propellers_json(propellers_json);

        std::vector<std::string> geometry_ids = propellers_json.at("geometry_ids").get<std::vector<std::string>>();
        double y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();
        double spin_inertia = 0.0;

        for (const std::string& geometry_id : geometry_ids) {
            const structural::Geometry& geom = structural_properties.get_geometry(geometry_id);
            y_min = std::min(y_min, geom.p_ref(1) - 0.5 * geom.y_size);
            y_max = std::max(y_max, geom.p_ref(1) + 0.5 * geom.y_size);
            spin_inertia += structural_properties.compute_spin_inertia(geom, n_prop);
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

    actuators::ActuatorProperties parse_actuator_properties(const nlohmann::json& config, structural::StructuralProperties& structural_properties) {
        const auto& surface_actuator_json = config.at("surfaces");
        const auto& propulsor_actuator_json = config.at("propulsors");

        actuators::SurfaceActuators surface_actuators = {
            .elevator = parse_surface_actuator<actuators::Elevator>(surface_actuator_json, "elevator"),
            .aileron = parse_surface_actuator<actuators::Aileron>(surface_actuator_json, "aileron"),
            .rudder = parse_surface_actuator<actuators::Rudder>(surface_actuator_json, "rudder"),
            .flap = parse_surface_actuator<actuators::Flap>(surface_actuator_json, "flap"),
            .spoiler = parse_surface_actuator<actuators::Spoiler>(surface_actuator_json, "spoiler")
        };

        actuators::PropulsorActuators propulsor_actuators = {
            .front_propulsor = parse_propulsor_actuator<actuators::FrontPropulsor>(propulsor_actuator_json, "front_propulsor", structural_properties),
            .left_propulsor = parse_propulsor_actuator<actuators::LeftPropulsor>(propulsor_actuator_json, "left_propulsor", structural_properties),
            .right_propulsor = parse_propulsor_actuator<actuators::RightPropulsor>(propulsor_actuator_json, "right_propulsor", structural_properties),
        };

        return { .surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators };
    }

}
