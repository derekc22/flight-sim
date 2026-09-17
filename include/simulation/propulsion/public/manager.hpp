#pragma once
#include "simulation/propulsion/public/components/collection.hpp"
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	struct PropulsionManager {
		PropulsorEffectors propulsor_effectors;

		/**
		 * @brief Computes the total propulsive wrench for one simulation step.
		 *
		 * Evaluates the front, left, and right propulsors and sums their body-expressed wrenches.
		 *
		 * @tparam T Scalar type used for the propulsion calculation.
		 * @param[in] input State, atmosphere, propulsor thrust commands, and timing data.
		 * @return Total propulsive force [N], moment [N m], and optional propeller angular velocities [rad/s].
		 */
		template <typename T> PropulsionManagerOutput_T<T> step(const PropulsionManagerInput_T<T>& input);

		/**
		 * @brief Commits current propeller angular velocities to the propulsor effectors.
		 *
		 * @param[in] propulsion_state_t Current angular velocity state for each propeller [rad/s].
		 */
		void commit(const PropulsionState& propulsion_state_t);
	};

} // namespace propulsion

#include "simulation/propulsion/public/manager.tpp"
