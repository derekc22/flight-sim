#pragma once
#include "simulation/frames/public/frame.hpp"

#include <Eigen/Dense>
#include <tuple>

namespace frames
{

	/**
	 * @brief Re-expresses a vector from one frame in another frame.
	 *
	 * @param[in] vA Vector expressed in frame @p A.
	 * @param[in] A Source frame in which @p vA is expressed.
	 * @param[in] B Destination frame in which the result is expressed.
	 * @return The same vector expressed in frame @p B, with the same units as @p vA.
	 */
	Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B);

	/**
	 * @brief Re-expresses a point from one frame in another frame.
	 *
	 * @param[in] pA Point coordinates expressed in frame @p A.
	 * @param[in] A Source frame in which @p pA is expressed.
	 * @param[in] B Destination frame in which the result is expressed.
	 * @return The same point expressed in frame @p B, with the same length units as @p pA.
	 */
	Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B);

	/**
	 * @brief Computes the homogeneous transformation from a reference frame to another frame.
	 *
	 * @param[in] F Destination frame.
	 * @param[in] R Reference frame.
	 * @return Homogeneous transformation from @p R to @p F.
	 */
	dynamics::HomogeneousTransformationMatrix H_from_R(const Frame& F, const Frame& R);

	/**
	 * @brief Computes the translational and angular velocity of a frame relative to a reference frame.
	 *
	 * @param[in] F Frame whose relative velocity is computed.
	 * @param[in] R Reference frame.
	 * @return Translational velocity [m/s] and angular velocity [rad/s] of @p F relative to @p R, expressed in
	 * @p F.
	 */
	std::tuple<dynamics::TranslationalVelocity, dynamics::AngularVelocity> vel_from_R(const Frame& F, const Frame& R);

} // namespace frames
