#pragma once
#include "simulation/frames/public/frame.hpp"

#include <Eigen/Dense>
#include <tuple>

namespace frames
{

	/** @brief Obtains the rotation matrix from an arbitrary root R to the frame F */
	Eigen::Matrix3d CRF(const Frame& F, const Frame& R);

	/** @brief Obtains the homogeneous transformation matrix from an arbitrary root R to the frame F */
	Eigen::Matrix4d HRF(const Frame& F, const Frame& R);

	/** @brief Obtains the displacement vector from an arbitrary root R to the frame F */
	Eigen::Vector3d pRF(const Frame& F, const Frame& R);

	std::tuple<dynamics::TranslationalVelocity, dynamics::AngularVelocity> vel_from_E(const Frame& F);

} // namespace frames
