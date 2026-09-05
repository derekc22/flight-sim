#include "simulation/trim/public/application.hpp"

#include <Eigen/Dense>

namespace trim
{

	dynamics::RigidBodyState update_state_from_trim(
		const dynamics::RigidBodyState& Xt,
		const dynamics::State_T<double>& trim_state)
	{
		dynamics::EulerAngles eul_curr;
		eul_curr.set(Xt.q);
		dynamics::EulerAngles eul_trim{Eigen::Vector3d(eul_curr.psi(), trim_state.theta, trim_state.phi)};
		dynamics::OrientationQuaternion qNB_trim;
		qNB_trim.set(eul_trim);

		dynamics::RigidBodyState Xt_trim = {
			.p = Xt.p,
			.v = dynamics::TranslationalVelocity{Eigen::Vector3d(trim_state.vx, trim_state.vy, trim_state.vz)},
			.q = qNB_trim,
			.w = dynamics::AngularVelocity{Eigen::Vector3d(trim_state.p, trim_state.q, trim_state.r)},
		};

		return Xt_trim;
	}

	void update_actuators_lag_from_trim(
		actuators::SurfaceActuators& surface_actuators,
		actuators::PropulsorActuators& propulsor_actuators,
		const TrimSolution& trim_sol)
	{
		const actuators::SurfaceActuatorInputs_T<double>& surface_inputs =
			trim_sol.operating_point.input.surface_inputs;
		const actuators::PropulsorActuatorInputs_T<double>& propulsor_inputs =
			trim_sol.operating_point.input.propulsor_inputs;

		surface_actuators.elevator.lag_state = surface_inputs.elevator_cmd;
		surface_actuators.aileron.lag_state = surface_inputs.aileron_cmd;
		surface_actuators.rudder.lag_state = surface_inputs.rudder_cmd;
		surface_actuators.flap.lag_state = surface_inputs.flap_cmd;
		surface_actuators.spoiler.lag_state = surface_inputs.spoiler_cmd;
		propulsor_actuators.front_propulsor.lag_state = propulsor_inputs.front_propulsor_cmd;
		propulsor_actuators.left_propulsor.lag_state = propulsor_inputs.left_propulsor_cmd;
		propulsor_actuators.right_propulsor.lag_state = propulsor_inputs.right_propulsor_cmd;
	}

} // namespace trim
