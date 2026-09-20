#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/integrators/public/data/types.hpp"

namespace integrators
{

	/**
	 * @brief Evaluates aerodynamic, propulsive, and net body-frame wrenches.
	 *
	 * @tparam T Scalar type used for the evaluation.
	 * @tparam Model Model type providing structural state and aerodynamic and propulsion managers.
	 * @param[in] model Structural state and aerodynamic and propulsion models.
	 * @param[in] twist Body-expressed translational velocity [m/s] and angular velocity [rad/s].
	 * @param[in] atm Static atmospheric state.
	 * @param[in] u Surface and propulsor actuator inputs.
	 * @param[in] windB Wind velocity expressed in body axes [m/s].
	 * @param[in] gB Gravity vector expressed in body axes [m/s^2].
	 * @param[in] dt Time step used by the propulsion model [s].
	 * @param[in] steady_state Whether to suppress transient propulsion effects.
	 * @return Aerodynamic, propulsive, and net body-frame wrenches plus the evaluated propulsion state.
	 */
	template <typename T, typename Model>
	WrenchEvaluation_T<T> compute_wrench_set_T(const Model& model,
		const dynamics::Twist_T<T>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const actuators::ActuatorInputs_T<T>& u,
		const atmospheric::Wind& windB,
		const constants::Vector3_T<T>& gB,
		T dt,
		bool steady_state);

} // namespace integrators

#include "simulation/integrators/public/detail/wrench.tpp"
