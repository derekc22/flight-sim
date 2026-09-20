#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"

namespace frames
{
	struct Frame;
}

namespace aerodynamics
{

	/**
	 * @brief Computes the aerodynamic state of one frame relative to another.
	 *
	 * @param[in] F Frame whose aerodynamic state is computed.
	 * @param[in] R Reference frame for the relative motion.
	 * @param[in] windB Wind velocity expressed in body axes [m/s].
	 * @return Freestream speed [m/s], angle of attack [rad], and sideslip angle [rad].
	 */
	AerodynamicState
	compute_aerodynamic_state(const frames::Frame& F, const frames::Frame& R, const atmospheric::Wind& windB);

	/**
	 * @brief Computes aerodynamic state from a body-expressed twist and wind velocity.
	 *
	 * @tparam T Scalar type used for the computation.
	 * @param[in] twist Body-expressed translational velocity [m/s] and angular velocity [rad/s].
	 * @param[in] windB Wind velocity expressed in body axes [m/s].
	 * @return Freestream speed [m/s], angle of attack [rad], and sideslip angle [rad].
	 */
	template <typename T>
	AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist,
		const atmospheric::Wind& windB);

	/**
	 * @brief Computes aerodynamic state from a rigid-body state and wind velocity.
	 *
	 * @param[in] X Rigid-body state containing body-expressed velocity.
	 * @param[in] windB Wind velocity expressed in body axes [m/s].
	 * @return Freestream speed [m/s], angle of attack [rad], and sideslip angle [rad].
	 */
	AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& X, const atmospheric::Wind& windB);

} // namespace aerodynamics

#include "simulation/aerodynamics/public/detail/state.tpp"
