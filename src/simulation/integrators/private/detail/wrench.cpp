#include <Eigen/Dense>
#include "simulation/integrators/private/detail/wrench.hpp"
#include "simulation/integrators/public/detail/wrench.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/geography/public/detail/gravity.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace integrators {

	WrenchEvaluation compute_net_wrench(
	    const dynamics::RigidBodyState& Xt,
	    RK4Model& model,
	    const operating::OperatingConditions& conditions,
	    const actuators::ActuatorInputs_T<double>& u,
	    double dt)
	{
		const dynamics::Twist_T<double> twist{.v = Xt.v.data, .w = Xt.w.data};

		const atmospheric::Wind windB{Xt.q.data * conditions.windI.data};
		const Eigen::Vector3d gB = geography::gB(Xt.q).data;

		const WrenchEvaluation_T<double> evaluation =
		    compute_wrench_set_T<double>(model, twist, conditions.atm, u, windB, gB, dt, conditions.steady_state);

		const dynamics::WrenchSet_T<double>& WB_set = evaluation.WB_set;

		return {
		    .WB_set = {.aerodynamic = {.F = dynamics::Force{WB_set.aerodynamic.F},
		                   .M = dynamics::Moment{WB_set.aerodynamic.M}},
		        .propulsive = {.F = dynamics::Force{WB_set.propulsive.F}, .M = dynamics::Moment{WB_set.propulsive.M}},
		        .net = {.F = dynamics::Force{WB_set.net.F}, .M = dynamics::Moment{WB_set.net.M}}},
		    .propulsion_state_t = evaluation.propulsion_state_t};
	}

} // namespace integrators
