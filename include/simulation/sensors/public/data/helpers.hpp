#pragma once
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

	SensorGroundTruth build_sensor_gt(const dynamics::RigidBodyState& Xt,
	    const dynamics::RigidBodyState& XEt,
	    const aerodynamics::AerodynamicState& aero_t,
	    const atmospheric::StaticAtmosphericState& atm_t,
	    const dynamics::Mass& mass,
	    const dynamics::Wrench& WB_net);

}
