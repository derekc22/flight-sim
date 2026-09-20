#pragma once
#include "simulation/frames/public/frame.hpp"

#include <Eigen/Dense>
#include <tuple>

namespace frames
{

	/**
	 * @brief Computes the orientation matrix from a reference frame to another frame.
	 *
	 * @param[in] F Destination frame.
	 * @param[in] R Reference frame.
	 * @return Orientation matrix CRF that re-expresses vectors from @p R in @p F [-].
	 */
	Eigen::Matrix3d CRF(const Frame& F, const Frame& R);

	/**
	 * @brief Computes the homogeneous transformation from a reference frame to another frame.
	 *
	 * @param[in] F Destination frame.
	 * @param[in] R Reference frame.
	 * @return Homogeneous transformation HRF that re-expresses points from @p R in @p F.
	 */
	Eigen::Matrix4d HRF(const Frame& F, const Frame& R);

	/**
	 * @brief Computes the position of a frame relative to a reference frame.
	 *
	 * @param[in] F Frame whose position is computed.
	 * @param[in] R Reference frame.
	 * @return Position of @p F relative to @p R, expressed in @p R [m].
	 */
	Eigen::Vector3d pRF(const Frame& F, const Frame& R);

	/**
	 * @brief Computes the translational and angular velocity of a frame relative to the root frame.
	 *
	 * @param[in] F Frame whose velocity is computed.
	 * @return Translational velocity [m/s] and angular velocity [rad/s] relative to the root, expressed in @p F.
	 */
	std::tuple<dynamics::TranslationalVelocity, dynamics::AngularVelocity> vel_from_E(const Frame& F);

} // namespace frames
