#include "simulation/sensors/public/data/helpers.hpp"

#include "simulation/atmospheric/public/detail/compressible.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/detail/gravity.hpp"

namespace sensors
{

	SensorGroundTruth build_sensor_gt(
		const dynamics::RigidBodyState& Xt,
		const dynamics::RigidBodyState& XEt,
		const aerodynamics::AerodynamicState& aero_t,
		const atmospheric::StaticAtmosphericState& atm_t,
		const dynamics::Mass& mass,
		const dynamics::Wrench& WB_net)
	{

		atmospheric::MachNumber Mach = atmospheric::mps_to_mach(Xt.v, atm_t.T);
		atmospheric::StagnationAtmosphericState atm0_t = atmospheric::static_to_stagnation(atm_t, Mach);
		dynamics::Gravity gB = geography::gB(XEt.p, XEt.q);
		dynamics::EulerAngles eul;
		eul.set(Xt.q);

		SensorGroundTruth sensor_gt = {
			.alpha = aero_t.alpha,
			.fB = dynamics::TranslationalAcceleration{WB_net.F.data / mass.data - gB.data},
			.wB_BI = Xt.w,
			.P0 = atm0_t.P0,
			.P = atm_t.P,
			.T0 = atm0_t.T0,
			.pI_BI = Xt.p,
			.vB_BI = Xt.v,
			.heading = geography::Heading{eul.psi()},
		};

		return sensor_gt;
	}

} // namespace sensors
