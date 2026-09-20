#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace dynamics
{

	/**
	 * @brief Unpacks a structured state into vector form.
	 *
	 * @tparam T Scalar type stored by the state.
	 * @param[in] x Structured state.
	 * @return State vector with velocity [m/s], angular velocity [rad/s], and attitude [rad] components.
	 */
	template <typename T> StateVector_T<T> unpack_state_T(const State_T<T>& x);

	/**
	 * @brief Packs a state vector into structured form.
	 *
	 * @tparam T Scalar type stored by the state.
	 * @param[in] x State vector with velocity [m/s], angular velocity [rad/s], and attitude [rad] components.
	 * @return Structured state.
	 */
	template <typename T> State_T<T> pack_state_T(const StateVector_T<T>& x);

	/**
	 * @brief Unpacks a structured state derivative into vector form.
	 *
	 * @tparam T Scalar type stored by the state derivative.
	 * @param[in] x_dot Structured state derivative.
	 * @return State-derivative vector with acceleration [m/s^2], angular acceleration [rad/s^2], and attitude-rate
	 * [rad/s] components.
	 */
	template <typename T> StateDotVector_T<T> unpack_state_dot_T(const StateDot_T<T>& x_dot);

	/**
	 * @brief Unpacks a structured wrench into vector form.
	 *
	 * @tparam T Scalar type stored by the wrench.
	 * @param[in] wrench Structured force [N] and moment [N m].
	 * @return Wrench vector with force followed by moment components.
	 */
	template <typename T> WrenchVector_T<T> unpack_wrench_T(const Wrench_T<T>& wrench);

	/**
	 * @brief Packs a wrench vector into structured form.
	 *
	 * @tparam T Scalar type stored by the wrench.
	 * @param[in] wrench Wrench vector with force [N] followed by moment [N m] components.
	 * @return Structured force and moment.
	 */
	template <typename T> Wrench_T<T> pack_wrench_T(const WrenchVector_T<T>& wrench);

	/**
	 * @brief Unpacks a wrench into vector form.
	 *
	 * @param[in] wrench Structured force [N] and moment [N m].
	 * @return Wrench vector with force followed by moment components.
	 */
	WrenchVector_T<double> unpack_wrench(const Wrench& wrench);

	/**
	 * @brief Packs a wrench vector into a wrench.
	 *
	 * @param[in] wrench Wrench vector with force [N] followed by moment [N m] components.
	 * @return Structured force and moment.
	 */
	Wrench pack_wrench(const WrenchVector_T<double>& wrench);

	/**
	 * @brief Converts a templated double-precision wrench to the concrete wrench type.
	 *
	 * @param[in] wrench Templated force [N] and moment [N m].
	 * @return Concrete force and moment.
	 */
	Wrench pack_wrench(const Wrench_T<double>& wrench);

	/**
	 * @brief Builds translational and angular velocity vectors from a structured state.
	 *
	 * @tparam T Scalar type stored by the state.
	 * @param[in] x Structured state containing body velocity and angular-velocity components.
	 * @return Twist containing translational velocity [m/s] and angular velocity [rad/s].
	 */
	template <typename T> Twist_T<T> build_twist_from_state_T(const State_T<T>& x);

	/**
	 * @brief Packs a rigid-body state into the reduced structured state.
	 *
	 * @param[in] Xt Rigid-body state.
	 * @return Structured velocity, angular velocity, roll, and pitch state.
	 */
	State_T<double> pack_state(const RigidBodyState& Xt);

	/**
	 * @brief Unpacks a rigid-body state into the reduced state vector.
	 *
	 * @param[in] Xt Rigid-body state.
	 * @return Reduced state vector with velocity [m/s], angular velocity [rad/s], and attitude [rad] components.
	 */
	StateVector_T<double> unpack_state(const RigidBodyState& Xt);

} // namespace dynamics

#include "simulation/dynamics/public/data/helpers.tpp"
