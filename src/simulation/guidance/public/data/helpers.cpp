#include "simulation/dynamics/public/data/helpers.hpp"

#include "simulation/constants/public/dimensions.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/helpers.hpp"

namespace guidance
{

	GuidanceSetpointVector unpack_guidance_setpoint(
		const dynamics::RigidBodyState& Xt)
	{
		dynamics::EulerAngles eulIB;
		eulIB.set(Xt.q);

		GuidanceSetpointVector out;
		out.head<constants::state_dim>() = dynamics::unpack_state(Xt);
		out(constants::state_dim) = eulIB.psi();

		return out;
	}

} // namespace guidance
