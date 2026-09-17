#pragma once

#include <Eigen/Dense>
#include <string>

namespace structural
{

	struct Geometry {
		std::string id;
		double mass;
		double x_size;
		double y_size;
		double z_size;
		Eigen::Vector3d pB_geomB;
		Eigen::Matrix3d CBL;

		/**
		 * @brief Computes the geometry's inertia tensor about its center in local axes.
		 *
		 * Models the geometry as a rectangular prism aligned with its local axes.
		 *
		 * @return Local inertia tensor about the geometry center [kg m^2].
		 */
		Eigen::Matrix3d compute_local_J();

		/**
		 * @brief Computes the geometry's moment of inertia about a body-expressed spin axis.
		 *
		 * @param[in] axis Spin axis expressed in body axes; its magnitude is ignored [-].
		 * @return Moment of inertia about the normalized spin axis [kg m^2].
		 */
		double compute_spin_inertia(const Eigen::Vector3d& axis);
	};

} // namespace structural
