#include "core/json/guidance/private/validation.hpp"

#include "simulation/util/public/validation.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace json
{

	void validate_attitude_control_setpoint(
		const nlohmann::json& guidance_json,
		const control::ControllerType& controller_type)
	{
		if (controller_type == control::ControllerType::DamperPID) {
			if (guidance_json.contains("v")) {
				throw std::runtime_error("json::validate_attitude_control_setpoint: DamperPID does not require v");
			}
			if (guidance_json.contains("w")) {
				throw std::runtime_error("json::validate_attitude_control_setpoint: DamperPID does not require w");
			}
			if (guidance_json.contains("eul")) {
				throw std::runtime_error("json::validate_attitude_control_setpoint: DamperPID does not require eul");
			}
		}

		if (controller_type == control::ControllerType::AttitudePID) {
			if (guidance_json.contains("v")) {
				throw std::runtime_error("json::validate_attitude_control_setpoint: AttitudePID does not require v");
			}
			if (!guidance_json.contains("w")) {
				throw std::runtime_error("json::validate_attitude_control_setpoint: AttitudePID requires w");
			}
			if (!guidance_json.contains("eul")) {
				throw std::runtime_error("json::validate_attitude_control_setpoint: AttitudePID requires eul");
			}
		}
	}

	void validate_velocity_control_setpoint(
		const nlohmann::json& guidance_json,
		const control::ControllerType& controller_type)
	{
		if (controller_type == control::ControllerType::VelocityPID) {
			if (!guidance_json.contains("v")) {
				throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID requires v");
			}
			if (guidance_json.contains("w")) {
				throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID does not require w");
			}
			if (guidance_json.contains("eul")) {
				throw std::runtime_error("json::validate_velocity_control_setpoint: VelocityPID does not require eul");
			}
		}
	}

	void validate_linear_quadratic_control_setpoint(
		const nlohmann::json& guidance_json,
		const control::ControllerType& controller_type)
	{
		if (controller_type == control::ControllerType::LinearQuadraticRegulator ||
			controller_type == control::ControllerType::LinearQuadraticTracker ||
			controller_type == control::ControllerType::LinearQuadraticIntegrator) {
			if (!guidance_json.contains("v")) {
				throw std::runtime_error(
					"json::validate_linear_quadratic_control_setpoint: linear_quadratic requires v");
			}
			if (!guidance_json.contains("w")) {
				throw std::runtime_error(
					"json::validate_linear_quadratic_control_setpoint: linear_quadratic requires w");
			}
			if (!guidance_json.contains("eul")) {
				throw std::runtime_error(
					"json::validate_linear_quadratic_control_setpoint: linear_quadratic requires eul");
			}
		}
	}

	void validate_nonlinear_control_setpoint(
		const nlohmann::json& guidance_json,
		const control::ControllerType& controller_type)
	{
		if (controller_type == control::ControllerType::FeedbackLinearization ||
			controller_type == control::ControllerType::NonlinearDynamicInversion ||
			controller_type == control::ControllerType::IncrementalNonlinearDynamicInversion) {
			if (!guidance_json.contains("v")) {
				throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires v");
			}
			if (!guidance_json.contains("w")) {
				throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires w");
			}
			if (!guidance_json.contains("eul")) {
				throw std::runtime_error("json::validate_nonlinear_control_setpoint: nonlinear requires eul");
			}
		}
	}

	void validate_guidance(
		const nlohmann::json& guidance_json)
	{
		bool attitude_flag = guidance_json.contains("attitude");
		bool velocity_flag = guidance_json.contains("velocity");
		bool linear_quadratic_flag = guidance_json.contains("linear_quadratic");
		bool nonlinear_flag = guidance_json.contains("nonlinear");

		if (attitude_flag && linear_quadratic_flag) {
			throw std::runtime_error(
				"json::validate_guidance: attitude and linear_quadratic guidance cannot both be present");
		}
		if (attitude_flag && nonlinear_flag) {
			throw std::runtime_error("json::validate_guidance: attitude and nonlinear guidance cannot both be present");
		}

		if (linear_quadratic_flag && nonlinear_flag) {
			throw std::runtime_error(
				"json::validate_guidance: linear_quadratic and nonlinear guidance cannot both be present");
		}

		if (velocity_flag && linear_quadratic_flag) {
			throw std::runtime_error(
				"json::validate_guidance: velocity and linear_quadratic guidance cannot both be present");
		}
		if (velocity_flag && nonlinear_flag) {
			throw std::runtime_error("json::validate_guidance: velocity and nonlinear guidance cannot both be present");
		}
	}

	void validate_trajectory_type(
		const guidance::TrajectoryComponents& traj_components,
		guidance::TrajectoryType traj_type)
	{
		if (traj_type == guidance::TrajectoryType::Regulation && traj_components.n_rows > 1) {
			throw std::runtime_error(
				"json::validate_trajectory_type: regulation trajectory cannot have more than one row");
		}
		if (traj_type == guidance::TrajectoryType::Interpolated && traj_components.n_rows != 2) {
			throw std::runtime_error("json::validate_trajectory_type: interpolated trajectory requires two rows");
		}
	}

	void validate_trajectory_components(
		guidance::TrajectoryComponents& traj_components)
	{
		int n_cols = 3; // v, w, and eul are 3-column vectors
		int n_rows = traj_components.n_rows;
		if (n_rows <= 0) {
			throw std::runtime_error("json::validate_trajectory_components: trajectory must have at least one row");
		}

		std::string context = "guidance::validate_trajectory_components";
		util::validate_shape(traj_components.v_traj, n_rows, n_cols, context, "v_traj");
		util::validate_shape(traj_components.w_traj, n_rows, n_cols, context, "w_traj");
		util::validate_shape(traj_components.eul_traj, n_rows, n_cols, context, "eul_traj");

		traj_components.n_rows = n_rows;
	}

} // namespace json
