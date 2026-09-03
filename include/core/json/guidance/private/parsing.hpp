#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "simulation/control/public/manager.hpp"
#include "simulation/guidance/public/data/types.hpp"
#include "simulation/guidance/public/manager.hpp"

namespace json {

	guidance::TrajectoryComponents& parse_attitude_control_setpoint(const nlohmann::json& guidance_json,
	    const control::ControllerType& controller_type,
	    guidance::TrajectoryComponents& traj_components);
	guidance::TrajectoryComponents& parse_velocity_control_setpoint(const nlohmann::json& guidance_json,
	    const control::ControllerType& controller_type,
	    guidance::TrajectoryComponents& traj_components);
	guidance::TrajectoryComponents& parse_linear_quadratic_control_setpoint(const nlohmann::json& guidance_json,
	    const control::ControllerType& controller_type,
	    guidance::TrajectoryComponents& traj_components);
	guidance::TrajectoryComponents& parse_nonlinear_control_setpoint(const nlohmann::json& guidance_json,
	    const control::ControllerType& controller_type,
	    guidance::TrajectoryComponents& traj_components);
	guidance::TrajectoryType map_trajectory_type(const std::string& trajectory_type_str);
	void fill_missing_trajectory_components(guidance::TrajectoryComponents& traj_components);
	guidance::Trajectory merge_trajectory_components(const guidance::TrajectoryComponents& traj_components);
	guidance::GuidanceManager parse_guidance_manager(const nlohmann::json& config,
	    const control::ControlManager& control_manager);

} // namespace json
