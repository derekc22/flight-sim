#pragma once
#include "simulation/avionics/public/data/types.hpp"

namespace avionics
{

	/**
	 * @brief Builds the ground-truth quantities derived by the avionics suite.
	 *
	 * Computes Mach number, body-expressed gravity, and Earth-radial vertical
	 * speed, while copying temperature, orientation, free-stream speed, geometric
	 * altitude, and density from the supplied simulation data.
	 *
	 * @param[in] Xt Rigid-body state supplying airspeed and orientation.
	 * @param[in] XEt Earth-referenced rigid-body state used for gravity and vertical speed.
	 * @param[in] aero_t Current aerodynamic state supplying free-stream speed.
	 * @param[in] atm_t Current static atmospheric state.
	 * @param[in] geo_t Current geographic state supplying geometric altitude.
	 * @return Ground-truth input for the modeled avionics calculations.
	 */
	AvionicsGroundTruth build_avionics_gt(const dynamics::RigidBodyState& Xt,
		const dynamics::RigidBodyState& XEt,
		const aerodynamics::AerodynamicState& aero_t,
		const atmospheric::StaticAtmosphericState& atm_t,
		const geography::GeographicState& geo_t);

	/**
	 * @brief Assembles a measured rigid-body state from sensor and avionics outputs.
	 *
	 * Selects INS position and velocity when @c use_ins is enabled; otherwise,
	 * it selects the GNSS measurements. Orientation always comes from the AHRS,
	 * and angular velocity always comes from the gyroscope.
	 *
	 * @param[in] sensor_meas Direct sensor measurements.
	 * @param[in] avionics_meas Derived avionics measurements.
	 * @param[in] avionics_settings Selection settings for state reconstruction.
	 * @return Combined measured rigid-body state.
	 */
	dynamics::RigidBodyState get_state_from_avionics(const sensors::SensorMeasurements& sensor_meas,
		const AvionicsMeasurements& avionics_meas,
		const Settings& avionics_settings);

} // namespace avionics
