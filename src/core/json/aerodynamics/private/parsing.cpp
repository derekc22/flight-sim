#include "core/json/aerodynamics/private/parsing.hpp"

#include "core/json/aerodynamics/private/validation.hpp"
#include "core/json/public/data/helpers.hpp"
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/aerodynamics/public/manager.hpp"
#include "simulation/util/public/validation.hpp"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace json
{

	aerodynamics::DynamicDerivatives parse_dynamic_derivatives(
		const nlohmann::json& dyn_json)
	{
		return {
			.CL_qhat = dyn_json.value("CL_qhat", 0.0),
			.CD_qhat = dyn_json.value("CD_qhat", 0.0),
			.CM_qhat = dyn_json.value("CM_qhat", 0.0),
			.CL_phat = dyn_json.value("CL_phat", 0.0),
			.CD_phat = dyn_json.value("CD_phat", 0.0),
			.CM_phat = dyn_json.value("CM_phat", 0.0),
			.CL_rhat = dyn_json.value("CL_rhat", 0.0),
			.CD_rhat = dyn_json.value("CD_rhat", 0.0),
			.CM_rhat = dyn_json.value("CM_rhat", 0.0),
		};
	}

	actuators::SurfaceActuatorID parse_surface_actuator_id(
		const std::string& actuator_id)
	{
		if (actuator_id == "elevator") {
			return actuators::SurfaceActuatorID::Elevator;
		}
		if (actuator_id == "aileron") {
			return actuators::SurfaceActuatorID::Aileron;
		}
		if (actuator_id == "rudder") {
			return actuators::SurfaceActuatorID::Rudder;
		}
		if (actuator_id == "flap") {
			return actuators::SurfaceActuatorID::Flap;
		}
		if (actuator_id == "spoiler") {
			return actuators::SurfaceActuatorID::Spoiler;
		}
		throw std::runtime_error("json::parse_surface_actuator_id: unknown actuator_id '" + actuator_id + "'");
	}

	aerodynamics::SurfaceEffector parse_surface_effector(
		const nlohmann::json& effector_json)
	{
		return {
			.actuator_id = parse_surface_actuator_id(effector_json.at("actuator_id").get<std::string>()),
			.dCL = effector_json.at("dCL").get<double>(),
			.dCD = effector_json.at("dCD").get<double>(),
			.dCM = effector_json.at("dCM").get<double>()
		};
	}

	std::vector<aerodynamics::SurfaceEffector> parse_surface_effectors(
		const nlohmann::json& effectors_json)
	{
		validate_surface_effectors_json(effectors_json);
		std::vector<aerodynamics::SurfaceEffector> effectors;
		effectors.reserve(effectors_json.size());
		for (const auto& effector_json : effectors_json) {
			effectors.push_back(parse_surface_effector(effector_json));
		}
		return effectors;
	}

	aerodynamics::AerodynamicsManager parse_aerodynamics_manager(
		const nlohmann::json& config)
	{
		const auto& surfaces_json = config.at("surfaces");
		validate_surfaces_json(surfaces_json);

		std::vector<aerodynamics::Surface> surfaces;
		surfaces.reserve(surfaces_json.size());
		for (const auto& surface_json : surfaces_json) {
			std::string id = surface_json.at("id").get<std::string>();
			Eigen::Vector3d pB_acB = parse_Vector3d(surface_json.at("pB_acB"));
			Eigen::Vector3d n = parse_Vector3d(surface_json.at("n"));
			util::validate_unit(n, "json::parse_aerodynamics_manager", id + " surface normal");

			surfaces.push_back(aerodynamics::Surface{
				.id = id,
				.chord = surface_json.at("chord").get<double>(),
				.span = surface_json.at("span").get<double>(),
				.pB_acB = pB_acB,
				.n = n,
				.CL0 = surface_json.at("CL0").get<double>(),
				.e = surface_json.at("e").get<double>(),
				.i = surface_json.at("i").get<double>(),
				.CD0 = surface_json.at("CD0").get<double>(),
				.CDa = surface_json.at("CDa").get<double>(),
				.a0 = surface_json.at("a0").get<double>(),
				.CM0 = surface_json.at("CM0").get<double>(),
				.CMa = surface_json.at("CMa").get<double>(),
				.dyn = surface_json.contains("dynamic_derivatives")
					? parse_dynamic_derivatives(surface_json.at("dynamic_derivatives"))
					: aerodynamics::DynamicDerivatives{},
				.effectors = surface_json.contains("effectors") ? parse_surface_effectors(surface_json.at("effectors"))
																: std::vector<aerodynamics::SurfaceEffector>{},
			});
		}
		return {surfaces};
	}

} // namespace json
