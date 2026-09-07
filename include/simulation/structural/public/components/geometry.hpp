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

		Eigen::Matrix3d compute_local_JB();
		double compute_spin_inertia(const Eigen::Vector3d& axis);
	};

} // namespace structural
