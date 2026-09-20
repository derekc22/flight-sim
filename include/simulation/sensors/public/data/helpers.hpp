#pragma once
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	/**
	 * @brief Builds the ground-truth quantities measured by the sensor suite.
	 *
	 * @param[in] Xt Rigid-body state supplying measured position, velocity, orientation, and angular velocity.
	 * @param[in] XEt Earth-referenced rigid-body state used to compute body-expressed gravity.
	 * @param[in] aero_t Current aerodynamic state supplying angle of attack.
	 * @param[in] atm_t Current static atmospheric state.
	 * @param[in] mass Aircraft mass [kg].
	 * @param[in] WB_net Net body-frame wrench supplying the force used for specific acceleration [N].
	 * @return Ground-truth input for every modeled sensor.
	 */
	SensorGroundTruth build_sensor_gt(const dynamics::RigidBodyState& Xt,
		const dynamics::RigidBodyState& XEt,
		const aerodynamics::AerodynamicState& aero_t,
		const atmospheric::StaticAtmosphericState& atm_t,
		const dynamics::Mass& mass,
		const dynamics::Wrench& WB_net);

} // namespace sensors
