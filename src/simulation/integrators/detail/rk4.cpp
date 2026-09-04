#include "simulation/integrators/private/detail/rk4.hpp"

#include "simulation/integrators/private/detail/wrench.hpp"
#include "simulation/integrators/public/detail/rk4.hpp"

namespace integrators
{

	RK4Output step_rigid_body_rk4(
		const dynamics::RigidBodyState& Xt,
		RK4Model& model,
		const operating::OperatingConditions& conditions,
		const actuators::ActuatorInputs_T<double>& u,
		double dt)
	{
		const dynamics::Mass mass = model.struc_t.mass;
		const dynamics::InertiaTensor JB = model.struc_t.JB;

		const WrenchEvaluation evaluation_1 = compute_net_wrench(Xt, model, conditions, u, dt);
		const dynamics::WrenchSet& wrench_1 = evaluation_1.WB_set;
		const dynamics::RigidBodyStateDot k1 = compute_rigid_body_state_dot(Xt, mass, JB, wrench_1.net);

		const dynamics::RigidBodyState X2 = add_scaled_rigid_body_state_dot(Xt, k1, 0.5 * dt);
		const WrenchEvaluation evaluation_2 = compute_net_wrench(X2, model, conditions, u, dt);
		const dynamics::WrenchSet& wrench_2 = evaluation_2.WB_set;
		const dynamics::RigidBodyStateDot k2 = compute_rigid_body_state_dot(X2, mass, JB, wrench_2.net);

		const dynamics::RigidBodyState X3 = add_scaled_rigid_body_state_dot(Xt, k2, 0.5 * dt);
		const WrenchEvaluation evaluation_3 = compute_net_wrench(X3, model, conditions, u, dt);
		const dynamics::WrenchSet& wrench_3 = evaluation_3.WB_set;
		const dynamics::RigidBodyStateDot k3 = compute_rigid_body_state_dot(X3, mass, JB, wrench_3.net);

		const dynamics::RigidBodyState X4 = add_scaled_rigid_body_state_dot(Xt, k3, dt);
		const WrenchEvaluation evaluation_4 = compute_net_wrench(X4, model, conditions, u, dt);
		const dynamics::WrenchSet& wrench_4 = evaluation_4.WB_set;
		const dynamics::RigidBodyStateDot k4 = compute_rigid_body_state_dot(X4, mass, JB, wrench_4.net);

		const dynamics::RigidBodyState Xt1 = add_rk4_weighted_rigid_body_state_dot(Xt, k1, k2, k3, k4, dt);

		model.propulsion.commit(evaluation_1.propulsion_state_t);

		return {.Xt1 = Xt1, .WB_set = wrench_1};
	}

} // namespace integrators
