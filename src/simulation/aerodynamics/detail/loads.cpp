#include "simulation/aerodynamics/public/detail/loads.hpp"

namespace aerodynamics
{

	dynamics::Wrench step_aero_forces_moments(
		const std::vector<Surface>& surfaces,
		const structural::CenterOfGravity& pB_GB,
		const dynamics::RigidBodyState& X,
		const atmospheric::StaticAtmosphericState& atm,
		const actuators::SurfaceActuatorInputs_T<double>& u,
		const atmospheric::Wind& windB)
	{
		const dynamics::Twist_T<double> twist{
			.v = X.v.data,
			.w = X.w.data,
		};

		const dynamics::Wrench_T<double> loads =
			step_aero_forces_moments_T<double>(surfaces, pB_GB.data, twist, atm, u, windB);

		return {dynamics::Force{loads.F}, dynamics::Moment{loads.M}};
	}

} // namespace aerodynamics
